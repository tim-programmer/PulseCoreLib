#!/usr/bin/env python3
"""Build, flash, run and verify the benchmark on an RP2040 board (Raspberry Pi Pico).

    .venv-tools/bin/python scripts/rp2040_bench.py [--type q15|q31|float] [--no-build]
                                                   [--no-flash] [--port /dev/ttyACM0]

1. builds the firmware (platform/rp2040, against the Pico SDK in $PICO_SDK_PATH, default
   /usr/share/pico-sdk) and the host reference (tools/bench_ref);
2. flashes it with picotool, which reboots a running Pico into BOOTSEL over USB (no button);
3. sends 'R' on the Pico's USB serial port and captures the streamed cases;
4. compares them with the host build exactly as scripts/mcxn947_bench.py does (bit-exact for
   Q15/Q31, a small tolerance for float) and prints measured cycles per sample.

Exit status 0 means every case arrived intact and matched.
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mcxn947_bench as common  # noqa: E402  (parse, report, run, ROOT)

ROOT = common.ROOT
PORT_GLOB = "/dev/serial/by-id/*Raspberry_Pi_Pico*if00"


def build(kind, sdk):
    t = kind.upper()
    common.run(["cmake", "-S", "platform/rp2040", "-B", f"build-rp2040-{kind}",
                f"-DPCL_SAMPLE_TYPE={t}", f"-DPICO_SDK_PATH={sdk}"], stdout=subprocess.DEVNULL)
    common.run(["cmake", "--build", f"build-rp2040-{kind}", "-j"], stdout=subprocess.DEVNULL)
    common.run(["cmake", "-B", f"build-{kind}", f"-DPCL_SAMPLE_TYPE={t}"], stdout=subprocess.DEVNULL)
    common.run(["cmake", "--build", f"build-{kind}", "--target", "bench_ref"],
               stdout=subprocess.DEVNULL)


def find_port(explicit, timeout=15.0):
    if explicit:
        return explicit
    deadline = time.time() + timeout
    while time.time() < deadline:  # the port comes and goes while the Pico reboots
        ports = sorted(glob.glob(PORT_GLOB))
        if ports:
            return ports[0]
        time.sleep(0.2)
    sys.exit("no Pico USB serial port found; is the bench firmware running?")


def capture(port):
    for _ in range(50):  # just after enumeration the port may refuse to open for a moment
        try:
            ser = serial.Serial(port, 115200, timeout=0.2)
            break
        except serial.SerialException:
            time.sleep(0.2)
    else:
        sys.exit(f"cannot open {port}")
    with ser:
        deadline = time.time() + 10
        data = b""
        while b"PCLBENCH1" not in data or not data.endswith(b"\n"):
            if time.time() > deadline:
                sys.exit("no banner from the Pico (is the bench firmware running?)")
            data += ser.read(256)
        ser.write(b"R")
        deadline = time.time() + 600  # the M0+ is slow, float especially (soft-float)
        while b"DONE\n" not in data:
            if time.time() > deadline:
                sys.exit(f"timed out waiting for DONE ({len(data)} bytes received)")
            data += ser.read(4096)
    # Start at the last hardware line + banner before the frames (the banner repeats while idle).
    done = data.index(b"DONE\n")
    pairs = list(re.finditer(rb"HPCLHW[^\n]*\nPCLBENCH1", data[:done]))
    first = pairs[-1].start() if pairs else data.rindex(b"PCLBENCH1", 0, done)
    return data[first:done + 5]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--type", choices=("q15", "q31", "float"), default="q15")
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--no-flash", action="store_true")
    ap.add_argument("--port")
    ap.add_argument("--sdk", default=os.environ.get("PICO_SDK_PATH", "/usr/share/pico-sdk"))
    args = ap.parse_args()
    kind = args.type

    print(f"== {kind.upper()} (RP2040) ==")
    if not args.no_build:
        print("build:")
        build(kind, args.sdk)
    ref_path = f"build-{kind}/pcl_bench_ref.bin"
    common.run([f"build-{kind}/tools/bench_ref", ref_path])
    ref_banner, ref_cases, sample_bytes = common.parse(
        open(os.path.join(ROOT, ref_path), "rb").read(), "host")

    if not args.no_flash:
        print("flash:")
        common.run(["picotool", "load", "-x", "-f", f"build-rp2040-{kind}/pcl_bench_rp2040.uf2"],
                   stdout=subprocess.DEVNULL)
        time.sleep(1.0)
    print("run:")
    raw = capture(find_port(args.port))
    return 0 if common.report(kind, raw, ref_banner, ref_cases, sample_bytes) else 1


if __name__ == "__main__":
    sys.exit(main())
