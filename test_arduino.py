import json
from juicer import Juicer
import time

PORT = "/dev/ttyACM0"  # Update this to your actual port (e.g., COM3 on Windows)


def send_get(ser, cmd):
    d = {"get": [cmd]}
    send(ser, d)
    resp = ser.readline().decode("utf-8").strip()
    return resp


def send_do(ser, **kwargs):
    d = {"do": kwargs}
    send(ser, d)


def send_set(ser, **kwargs):
    d = {"set": kwargs}
    send(ser, d)


def send(ser, d):
    print(f"Sending: {d}")
    msg = json.dumps(d, separators=(",", ":"), ensure_ascii=True) + "\n"
    ser.write(msg.encode("utf-8"))
    ser.flush()


def main():
    with Juicer(port="/dev/ttyACM0", baud=115200, settle_s=2.5) as j:
        _start_set = time.perf_counter()
        set_resp = j.set(flow_rate=0.5)
        _start_get = time.perf_counter()
        resp = j.get("flow_rate")
        _response_get = time.perf_counter()
        print(
            f"Set flow rate: {set_resp}, time = {_start_get - _start_set:.3f}s\n"
            f"Get flow rate: {resp}, time = {_response_get - _start_get:.3f}s\n------\n"
        )

        _start_rew = time.perf_counter()
        resp = j.reward_with_notify(0.1)
        _end_rew = time.perf_counter()
        print(f"Reward: {resp}, time: {_end_rew - _start_rew}")


if __name__ == "__main__":
    main()
