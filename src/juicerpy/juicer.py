"""
High-level Python helper for talking to the Juicer ESP32 pump over USB serial.

This module is intended for researchers to import into experiment scripts.

Key features:
- Newline-terminated JSON protocol at 2,000,000 baud (see `api.md`)
- Port auto-discovery (Windows/Linux) with explicit override
- Context-manager friendly
- By default, raises an exception when the device replies with status != "success"
- Reward notifications: request `notify` with `reward` to receive a follow-up JSON line
  (`reward_with_notify(...)` or `read(timeout_s)` after `reward(..., "notify")`)

Example:

```python
from juicer import Juicer

with Juicer() as j:
    j.set(target_rps=2.5)
    j.set(reward_overlap_policy="append")
    resp = j.reward(0.5, "reward_mls", "reward_number", "juice_level")
    print(resp)
```

Alternative (keep a connection open; remember to close):

```python
from juicer import Juicer

j = Juicer()
j.reward(0.5, "reward_mls", "reward_number", "juice_level")
j.close()
```
"""

from __future__ import annotations

import glob
import json
import time
from dataclasses import dataclass
from typing import Any, Iterable

import serial
from serial.tools import list_ports


BAUD_DEFAULT = 2_000_000


class JuicerError(RuntimeError):
    """Base error for this helper."""


class PortNotFound(JuicerError):
    pass


class ProtocolError(JuicerError):
    pass


class DeviceError(JuicerError):
    """Raised when the device replies with a failure status."""

    def __init__(self, message: str, response: dict[str, Any] | None = None):
        super().__init__(message)
        self.response = response


def find_juicer_port(preferred: str | None = None, *, debug: bool = False) -> str | None:
    """
    Best-effort port auto-discovery.

    Strategy (same as `test_connection.py` / `unit_test.py`):
    - match "juicer" in port description
    - match VID/PID 239A:8123
    - match VID/PID in hwid string
    - Linux: /dev/serial/by-id/*juicer*
    """
    if preferred:
        return preferred

    for p in list_ports.comports():
        desc = (p.description or "").lower()
        if "juicer" in desc:
            return p.device

    for p in list_ports.comports():
        if (p.vid == 0x239A) and (p.pid == 0x8123):
            return p.device

    for p in list_ports.comports():
        hwid = (p.hwid or "").lower()
        if ("vid:239a" in hwid or "vid_239a" in hwid or "vid=239a" in hwid) and (
            "pid:8123" in hwid or "pid_8123" in hwid or "pid=8123" in hwid or "239a:8123" in hwid
        ):
            return p.device

    for path in glob.glob("/dev/serial/by-id/*juicer*"):
        return path

    if debug:
        ports = list(list_ports.comports())
        print("Auto-detect tried description->vid/pid->hwid->by-id and found nothing.")
        for p in ports:
            print(f"- device={p.device}, desc={p.description!r}, vid={p.vid}, pid={p.pid}, hwid={p.hwid!r}")

    return None


def _compact_json(obj: dict[str, Any]) -> str:
    # Compact JSON to avoid overrunning the firmware's small RX buffer.
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=True)


@dataclass
class Juicer:
    """
    High-level device wrapper.

    Defaults are chosen to match working scripts in this repo.
    """

    port: str | None = None
    baud: int = BAUD_DEFAULT
    timeout_s: float = 3.0
    write_timeout_s: float = 3.0
    settle_s: float = 0.10
    dtr: bool = True
    rts: bool = True
    raise_on_failure: bool = True

    def __post_init__(self) -> None:
        if self.port is None:
            self.port = find_juicer_port()
        if not self.port:
            raise PortNotFound(
                "No juicer serial port found. Provide Juicer(port='...') or ensure the device is connected."
            )
        self._ser = self._open_serial(self.port)
        # Many USB-CDC stacks behave better with DTR/RTS asserted.
        for attr, val in (("dtr", self.dtr), ("rts", self.rts)):
            try:
                setattr(self._ser, attr, val)
            except Exception:
                pass
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass
        if self.settle_s > 0:
            time.sleep(self.settle_s)

    def _open_serial(self, port: str) -> serial.Serial:
        try:
            return serial.Serial(
                port,
                self.baud,
                timeout=self.timeout_s,
                write_timeout=self.write_timeout_s,
                dsrdtr=True,
                rtscts=False,
            )
        except Exception as e:
            raise JuicerError(f"Failed to open serial port {port!r}: {e!r}") from e

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass

    def __enter__(self) -> "Juicer":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _readline_json(self, deadline_s: float) -> dict[str, Any]:
        """
        Read until a newline, returning the first parseable JSON object found.
        Ignores non-JSON lines (e.g. boot noise) until the deadline.
        """
        original_timeout = self._ser.timeout
        try:
            while time.time() < deadline_s:
                remaining = max(0.0, deadline_s - time.time())
                self._ser.timeout = remaining
                line = self._ser.readline()
                if not line:
                    continue
                line_s = line.decode(errors="replace").strip()
                if not line_s:
                    continue
                if not line_s.startswith("{"):
                    continue
                try:
                    obj = json.loads(line_s)
                except json.JSONDecodeError:
                    continue
                if isinstance(obj, dict):
                    return obj
        finally:
            self._ser.timeout = original_timeout

        raise ProtocolError("Timed out waiting for a JSON response line from device")

    def read(self, timeout_s: float | None = None) -> dict[str, Any]:
        """
        Read the next JSON response line within the timeout.
        """
        if timeout_s is None:
            timeout_s = self.timeout_s
        return self._readline_json(time.time() + float(timeout_s))

    def request(
        self,
        payload: dict[str, Any],
        *,
        timeout_s: float | None = None,
        raise_on_failure: bool | None = None,
    ) -> dict[str, Any]:
        """
        Send a JSON command terminated by newline; return response dict.

        If raise_on_failure is enabled (default), raise DeviceError when the response includes a
        `status` field that is not "success" (e.g. {"status":"failure",...} or {"status":"Invalid JSON format"}).
        """
        if timeout_s is None:
            timeout_s = self.timeout_s
        if raise_on_failure is None:
            raise_on_failure = self.raise_on_failure

        msg = _compact_json(payload) + "\n"
        try:
            # Correlate response to request.
            self._ser.reset_input_buffer()
        except Exception:
            pass
        try:
            self._ser.write(msg.encode("utf-8"))
            self._ser.flush()
        except serial.SerialTimeoutException as e:
            raise ProtocolError(f"Serial write timed out (port busy/wedged?): {e!r}") from e

        resp = self._readline_json(time.time() + float(timeout_s))

        if raise_on_failure and "status" in resp and resp.get("status") != "success":
            # Prefer explicit error field when present.
            err = resp.get("error") or resp.get("status") or "Device reported failure"
            raise DeviceError(str(err), response=resp)

        return resp

    def get(self, *keys: str, timeout_s: float | None = None) -> dict[str, Any]:
        return self.request({"get": list(keys)}, timeout_s=timeout_s, raise_on_failure=False)

    def set(self, *, timeout_s: float | None = None, **kwargs: Any) -> dict[str, Any]:
        return self.request({"set": kwargs}, timeout_s=timeout_s)

    def do(self, action: Any, *, get: Iterable[str] | None = None, timeout_s: float | None = None) -> dict[str, Any]:
        payload: dict[str, Any] = {"do": action}
        if get is not None:
            payload["get"] = list(get)
        return self.request(payload, timeout_s=timeout_s)

    # Convenience methods
    def reward(self, mls: float, *get_keys: str, timeout_s: float | None = None) -> dict[str, Any]:
        """
        Dispense `mls` and optionally fetch additional keys in the same request.

        Example:
            j.reward(0.5, "reward_mls", "reward_number", "juice_level")
        """
        return self.do({"reward": float(mls)}, get=get_keys or None, timeout_s=timeout_s)

    def reward_with_notify(
        self,
        mls: float,
        *get_keys: str,
        timeout_s: float | None = None,
        notify_timeout_s: float | None = None,
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        """
        Dispense `mls` and wait for the follow-up notify line.

        Returns (response, notify_response). The notify response is typically:
        {"notify": "reward_complete"}.
        """
        get_list = list(get_keys)
        if "notify" not in get_list:
            get_list.append("notify")
        resp = self.do({"reward": float(mls)}, get=get_list, timeout_s=timeout_s)
        deadline_s = time.time() + float(notify_timeout_s or timeout_s or self.timeout_s)
        notify = self._readline_json(deadline_s)
        return resp, notify

    def purge(self, mls: float, *get_keys: str, timeout_s: float | None = None) -> dict[str, Any]:
        return self.do({"purge": float(mls)}, get=get_keys or None, timeout_s=timeout_s)

    def abort(self, *get_keys: str, timeout_s: float | None = None) -> dict[str, Any]:
        return self.do("abort", get=get_keys or None, timeout_s=timeout_s)

    def reset_counters(self, *get_keys: str, timeout_s: float | None = None) -> dict[str, Any]:
        return self.do("reset", get=get_keys or None, timeout_s=timeout_s)

    def adjust_flow_rate(self, *, expected_mls: float, actual_mls: float, timeout_s: float | None = None) -> dict[str, Any]:
        return self.request(
            {"set": {"adjust_flow_rate": {"expected_mls": float(expected_mls), "actual_mls": float(actual_mls)}}},
            timeout_s=timeout_s,
        )

