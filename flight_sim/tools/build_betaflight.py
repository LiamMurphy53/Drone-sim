#!/usr/bin/env python3
"""Rebuild the local simulator when its tracked transport patches change."""
import hashlib, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'vendor/betaflight-4.5.2'
BINARY = SOURCE/'obj/main/betaflight_SITL.elf'
STAMP = ROOT/'.tools/betaflight-patch.sha256'

def ensure_build():
    patcher = ROOT/'tools/patch_betaflight.py'
    digest = hashlib.sha256(patcher.read_bytes()).hexdigest()
    if BINARY.exists() and STAMP.exists() and STAMP.read_text().strip() == digest:
        return
    if not SOURCE.exists():
        raise RuntimeError('Betaflight sources are missing. Run tools/setup.sh first.')
    print('Updating the local simulator controller…', flush=True)
    subprocess.run([sys.executable,str(patcher)],check=True,cwd=ROOT)
    (SOURCE/'src/config/configs').mkdir(parents=True,exist_ok=True)
    (ROOT/'.tools').mkdir(exist_ok=True)
    log_path = ROOT/'.tools/betaflight-build.log'
    with log_path.open('w') as log:
        result = subprocess.run(['make','TARGET=SITL','ARM_SDK_DIR=/usr','CROSS_CC=clang',
            'LD_FLAGS=-lm -lpthread',
            'EXTRA_FLAGS=-Wno-error -Wno-unknown-warning-option -Wno-unused-command-line-argument',
            'obj/main/betaflight_SITL.elf','-j6'],cwd=SOURCE,stdout=log,stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError('Controller build failed; see '+str(log_path))
    STAMP.write_text(digest+'\n')

if __name__ == '__main__':
    ensure_build()
