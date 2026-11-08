#!/bin/sh
# One-time host setup for testing on a FRDM-MCXN947: a project-local Python venv with pyOCD
# and pyserial, the MCXN947 flash-algorithm pack, and the pinned NXP SDK sources.
# Everything lands in gitignored directories (.venv-tools/, third_party/).
set -eu
cd "$(dirname "$0")/.."

python3 -m venv .venv-tools
.venv-tools/bin/pip install --quiet --upgrade pip
.venv-tools/bin/pip install --quiet pyocd pyserial
.venv-tools/bin/pyocd pack install MCXN947VDF
./scripts/fetch_mcuxsdk.sh

echo
echo "Ready. Plug in the board's MCU-Link (debug) USB port, then:"
echo "  .venv-tools/bin/python scripts/mcxn947_bench.py --type q15"
