#!/usr/bin/env python3
"""Broadcast a lamp state to every lamp running `cyberlamp_fw_adv`, via BlueZ (Linux).

Advertises the state for a few seconds with `bluetoothctl`; no pairing or root needed.
Packet format: see `src/adv.rs`.

    python3 lamp_broadcast.py --mode sp --brightness 20 --mix 10
    python3 lamp_broadcast.py --mode wh --brightness 30 --seconds 10
"""
import argparse
import random
import subprocess
import sys
import time

COMPANY_ID = 0xFFFF  # reserved by the Bluetooth SIG for testing
MAGIC = b"CL"
VERSION = 1
GROUP_ALL = 0
MODES = {"wh": 0, "white": 0, "uv": 1, "ir": 2, "sp": 3, "spread": 3}
MAX_LEVEL = 40


def level(text):
    value = int(text)
    if not 0 <= value <= MAX_LEVEL:
        raise argparse.ArgumentTypeError(f"must be 0..{MAX_LEVEL}")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=sorted(MODES), required=True)
    parser.add_argument("--brightness", type=level, required=True)
    parser.add_argument("--mix", type=level, default=20, help="0 = cool, 40 = warm (white and spread)")
    parser.add_argument("--seconds", type=float, default=5, help="how long to advertise")
    parser.add_argument("--interval-ms", type=int, default=100, help="advertising interval")
    args = parser.parse_args()

    # Lamps apply a broadcast once per sequence number; a random one avoids repeating the
    # previous run's number.
    seq = random.randrange(256)
    payload = MAGIC + bytes([VERSION, GROUP_ALL, seq, MODES[args.mode], args.brightness, args.mix])
    data = " ".join(f"0x{b:02x}" for b in payload)  # bluetoothctl wants C literals
    print(f"Advertising mode={args.mode} brightness={args.brightness} mix={args.mix} seq={seq} "
          f"for {args.seconds} s")

    # The advertisement lives as long as this bluetoothctl session.
    ctl = subprocess.Popen(["bluetoothctl"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True)

    def command(line):
        ctl.stdin.write(line + "\n")
        ctl.stdin.flush()
        time.sleep(0.2)

    try:
        command("menu advertise")
        command(f"manufacturer 0x{COMPANY_ID:04x} {data}")
        command(f"interval {args.interval_ms} {args.interval_ms}")
        command("back")
        command("advertise broadcast")  # non-connectable
        time.sleep(args.seconds)
        command("advertise off")
        command("quit")
        output, _ = ctl.communicate(timeout=5)
    finally:
        if ctl.poll() is None:
            ctl.kill()
    if "Advertising object registered" not in output or "Invalid" in output:
        sys.exit("bluetoothctl did not advertise the lamp state:\n" + output)
    print("Done")


if __name__ == "__main__":
    main()
