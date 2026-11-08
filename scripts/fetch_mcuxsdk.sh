#!/bin/sh
# Fetches the parts of the NXP MCUXpresso SDK (release 24.12.00 manifest pins) that the
# MCX N947 firmware in platform/mcxn947 needs, into third_party/ (gitignored, never vendored).
# Blobless, sparse, pinned to exact commits, so it is small (~30 MB) and reproducible.
set -eu
cd "$(dirname "$0")/.."
mkdir -p third_party && cd third_party

fetch() { # name repo sha sparse-paths...
    name=$1 repo=$2 sha=$3; shift 3
    if [ ! -d "$name/.git" ]; then
        git init -q "$name"
        git -C "$name" remote add origin "https://github.com/nxp-mcuxpresso/$repo"
        git -C "$name" config core.sparseCheckout true
    fi
    printf '%s\n' "$@" > "$name/.git/info/sparse-checkout"
    if [ "$(git -C "$name" rev-parse -q --verify HEAD 2>/dev/null || true)" != "$sha" ]; then
        git -C "$name" fetch -q --depth 1 --filter=blob:none origin "$sha"
        git -C "$name" checkout -q --detach FETCH_HEAD
    else
        # Same commit, but the sparse paths may have changed: re-apply them.
        git -C "$name" read-tree -mu HEAD
    fi
    echo "$name @ $sha"
}

fetch mcuxsdk-core mcuxsdk-core 84547d55114471cd361e32559db1c4ecb3a32815 \
    /drivers/common/ /drivers/lpuart/ /drivers/mcx_spc/ /drivers/port/ /drivers/lpflexcomm/ \
    /drivers/gpio/ /drivers/syscon/ /COPYING-BSD-3 /MCUX_VERSION
fetch mcux-devices-mcx mcux-devices-mcx 3ff51c8cf729c59918e96f4ce33b37c3e93ffc74 \
    /MCXN/MCXN947/ /MCXN/periph/ /COPYING-BSD-3
fetch mcu-sdk-cmsis mcu-sdk-cmsis c3319a16588d05036a9e08cf59a92634a5a0e5dd \
    /Core/Include/ /LICENSE.txt
fetch mcuxsdk-examples mcuxsdk-examples 16bf470be9f9d7e66ea4ae832b3681e731773659 \
    /_boards/frdmmcxn947/board.h /_boards/frdmmcxn947/clock_config.c \
    /_boards/frdmmcxn947/clock_config.h /_boards/frdmmcxn947/demo_apps/hello_world/pin_mux.c \
    /_boards/frdmmcxn947/demo_apps/hello_world/pin_mux.h
