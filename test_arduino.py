from serial import Serial
import json
from juicer import Juicer

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
        j.set(flow_rate=0.5)
        resp = j.get("flow_rate")
        print(resp)
        resp = j.reward(0.1)
        print(resp)


if __name__ == "__main__":
    main()
