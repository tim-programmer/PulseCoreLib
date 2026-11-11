#!/usr/bin/env python3
"""Build, flash, run and verify the benchmark (oscillator, noise, S&H, VCA, LFO, ADSR, math, SVF, biquad, ladder, delay) on a FRDM-MCXN947.

    .venv-tools/bin/python scripts/mcxn947_bench.py [--type q15|q31|float] [--no-build]
                                                    [--no-flash] [--port /dev/ttyACM1]

1. builds the firmware (preset mcxn947-<type>) and the host reference (tools/bench_ref);
2. flashes the board through the on-board MCU-Link with pyOCD;
3. sends 'R' on the MCU-Link virtual serial port and captures the streamed cases;
4. checks each case's CRC and compares its samples with the host build (bit-exact for
   Q15/Q31, within a small tolerance for float) and prints measured cycles per sample.

Exit status 0 means every case arrived intact and matched.
"""
import argparse
import glob
import os
import re
import struct
import subprocess
import sys
import time
import zlib

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGET = "mcxn947vdf"
FRAMES = 2048
BLOCK = 256
FLOAT_TOL = 1e-6


def run(cmd, **kw):
    print("  $ " + " ".join(cmd))
    subprocess.run(cmd, cwd=ROOT, check=True, **kw)


def build(kind):
    t = kind.upper()
    run(["cmake", "--preset", f"mcxn947-{kind}"], stdout=subprocess.DEVNULL)
    run(["cmake", "--build", "--preset", f"mcxn947-{kind}"], stdout=subprocess.DEVNULL)
    run(["cmake", "-B", f"build-{kind}", f"-DPCL_SAMPLE_TYPE={t}"], stdout=subprocess.DEVNULL)
    run(["cmake", "--build", f"build-{kind}", "--target", "bench_ref"], stdout=subprocess.DEVNULL)


def find_port(explicit):
    if explicit:
        return explicit
    # The MCU-Link exposes the board's console UART as its third interface (if02).
    ports = sorted(glob.glob("/dev/serial/by-id/*MCU-LINK*FRDM-MCXN947*if02"))
    if not ports:
        sys.exit("no MCU-Link virtual serial port found; is the debug USB port connected?")
    return ports[0]


def pyocd(*args):
    return [sys.executable, "-m", "pyocd", *args, "-t", TARGET]


def parse(stream, source):
    """Parses banner lines and case frames from `stream` (bytes). Returns (banner, cases)."""
    parse.hw = None
    banner, cases, pos = None, {}, 0
    sample_bytes = None
    while pos < len(stream):
        tag = stream[pos:pos + 1]
        if tag == b" ":  # padding from the firmware's clock measurement
            pos += 1
        elif tag == b"H":  # hardware info line
            end = stream.index(b"\n", pos)
            parse.hw = stream[pos + 1:end].decode()
            pos = end + 1
        elif tag == b"P":  # banner line
            end = stream.index(b"\n", pos)
            banner = stream[pos:end].decode()
            for field in banner.split():
                if field.startswith("sample_bytes="):
                    sample_bytes = int(field.split("=")[1])
            pos = end + 1
        elif tag == b"C":
            if sample_bytes is None:
                sys.exit(f"{source}: frame before banner")
            cid = stream[pos + 1]
            frames, total, cmin, cmax = struct.unpack_from("<IIII", stream, pos + 2)
            start = pos + 18
            n = frames * sample_bytes
            payload = stream[start:start + n]
            (crc,) = struct.unpack_from("<I", stream, start + n)
            cases[cid] = dict(frames=frames, total=total, min=cmin, max=cmax, payload=payload,
                              crc_ok=(zlib.crc32(payload) & 0xFFFFFFFF) == crc)
            pos = start + n + 4
        elif tag == b"D":  # DONE
            pos = stream.index(b"\n", pos) + 1
        else:
            sys.exit(f"{source}: unexpected byte {tag!r} at offset {pos}")
    return banner, cases, sample_bytes


def capture(port, baud, reset_cmd):
    with serial.Serial(port, baud, timeout=0.2) as ser:
        ser.reset_input_buffer()
        subprocess.run(reset_cmd, cwd=ROOT, check=True, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        deadline = time.time() + 10
        data = b""
        while b"PCLBENCH1" not in data:
            if time.time() > deadline:
                sys.exit("no banner from the board (did it boot? wrong baud?)")
            data += ser.read(256)
        ser.write(b"R")
        deadline = time.time() + 60
        while b"DONE\n" not in data:
            if time.time() > deadline:
                sys.exit(f"timed out waiting for DONE ({len(data)} bytes received)")
            data += ser.read(4096)
    # The firmware starts printing its banner again right after DONE; drop anything past it.
    # Likewise anything before the first banner (the line glitches while the chip resets).
    # Start at a complete "hardware line + banner" pair if there is one (a glitch can corrupt
    # the first line the chip prints while it comes out of reset).
    pair = re.search(rb"HPCLHW[^\n]*\nPCLBENCH1", data)
    first = pair.start() if pair else data.index(b"PCLBENCH1")
    return data[first:data.index(b"DONE\n") + 5]


def case_names():
    """Case names in id order: the first quoted string of each *_CASE(...) line in the table."""
    text = open(os.path.join(ROOT, "platform/bench/pcl_bench.c")).read()
    table = text[text.index("g_cases[] = {"):]
    table = table[:table.index("};")]
    return re.findall(r'^\s*[A-Z]+_CASE\("([^"]+)"', table, re.M)


def decode(payload, sample_bytes, kind):
    if kind == "float":
        return struct.unpack(f"<{len(payload) // 4}f", payload)
    fmt = "h" if sample_bytes == 2 else "i"
    return struct.unpack(f"<{len(payload) // sample_bytes}{fmt}", payload)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--type", choices=("q15", "q31", "float"), default="q15")
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--no-flash", action="store_true")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=921600)
    args = ap.parse_args()
    kind = args.type

    print(f"== {kind.upper()} ==")
    if not args.no_build:
        print("build:")
        build(kind)
    elf = f"build-mcxn947-{kind}/platform/mcxn947/pcl_bench_mcxn947.elf"
    ref_path = f"build-{kind}/pcl_bench_ref.bin"
    run([f"build-{kind}/tools/bench_ref", ref_path])
    ref_banner, ref_cases, sample_bytes = parse(open(os.path.join(ROOT, ref_path), "rb").read(), "host")

    if not args.no_flash:
        print("flash:")
        run(pyocd("flash", elf), stdout=subprocess.DEVNULL)
    print("run:")
    port = find_port(args.port)
    raw = capture(port, args.baud, pyocd("reset"))
    banner, cases, _ = parse(raw, "board")
    print(f"  board: {banner}")
    if parse.hw:
        print(f"  {parse.hw}")
    if banner is None or banner.split()[1:3] != ref_banner.split()[1:3]:
        sys.exit(f"board build does not match host reference: {banner!r} vs {ref_banner!r}")

    names = case_names()
    ok = len(cases) == len(ref_cases)
    print(f"\n{'case':<15}{'crc':<5}{'samples':<24}{'cyc/sample':>11}{'(min blk)':>11}{'max blk':>9}")
    for cid in sorted(ref_cases):
        r, b = ref_cases[cid], cases.get(cid)
        name = names[cid] if cid < len(names) else f"case{cid}"
        if b is None:
            print(f"{name:<15}MISSING")
            ok = False
            continue
        if kind == "float":
            rs, bs = decode(r["payload"], sample_bytes, kind), decode(b["payload"], sample_bytes, kind)
            worst = max(abs(x - y) for x, y in zip(rs, bs))
            exact = sum(x == y for x, y in zip(rs, bs))
            match = worst <= FLOAT_TOL
            verdict = f"{'ok' if match else 'DIFF'} max|d|={worst:.1e} ({exact}/{len(rs)} exact)"
        else:
            diff = sum(x != y for x, y in zip(r["payload"], b["payload"]))
            match = r["payload"] == b["payload"]
            verdict = "bit-exact" if match else f"DIFF ({diff} bytes)"
        good = b["crc_ok"] and match
        ok &= good
        print(f"{name:<15}{'ok' if b['crc_ok'] else 'BAD':<5}{verdict:<24}"
              f"{b['total'] / b['frames']:>11.1f}{b['min'] / BLOCK:>11.1f}{b['max'] / BLOCK:>9.1f}")
    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
