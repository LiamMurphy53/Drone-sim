#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p .tools vendor runtime
if [[ ! -x .tools/Godot.app/Contents/MacOS/Godot ]] || [[ "$(.tools/Godot.app/Contents/MacOS/Godot --version)" != 4.5.2.* ]]; then
  # 4.5+ uses SDL on macOS, including generic EdgeTX USB joysticks.
  godot_stage=$(mktemp -d .tools/godot-install.XXXXXX)
  trap 'rm -rf "$godot_stage"' EXIT
  curl -fL --retry 2 -o "$godot_stage/godot.zip" 'https://github.com/godotengine/godot-builds/releases/download/4.5.2-stable/Godot_v4.5.2-stable_macos.universal.zip'
  echo "2a3f35cf5813b0d26e3f4c15dabc5e7c58407fceec7bae5291740772f72d141a  $godot_stage/godot.zip" | shasum -a 256 -c -
  unzip -q "$godot_stage/godot.zip" -d "$godot_stage"
  if [[ -d .tools/Godot.app ]]; then
    mv .tools/Godot.app ".tools/Godot-backup-$(date +%s)-$$.app"
  fi
  mv "$godot_stage/Godot.app" .tools/Godot.app
  mv "$godot_stage/godot.zip" .tools/godot.zip
  rmdir "$godot_stage"
  trap - EXIT
fi
if [[ ! -d vendor/betaflight-4.5.2 ]]; then
  curl -fL --retry 2 -o .tools/betaflight.tar.gz 'https://github.com/betaflight/betaflight/archive/refs/tags/4.5.2.tar.gz'
  echo '5660582e32956c5a483a7529413c40df8d6fdaafcbdea4cc501d81da899832b5  .tools/betaflight.tar.gz' | shasum -a 256 -c -
  tar -xzf .tools/betaflight.tar.gz -C vendor
fi
python3 tools/patch_betaflight.py
touch vendor/.gdignore runtime/.gdignore
mkdir -p vendor/betaflight-4.5.2/src/config/configs
(
  cd vendor/betaflight-4.5.2
  # SITL needs the host compiler, not the ARM firmware toolchain. The existing
  # Apple parameter-group sections work with the Mach-O linker without pg.ld.
  make TARGET=SITL ARM_SDK_DIR=/usr CROSS_CC=clang LD_FLAGS='-lm -lpthread' \
    EXTRA_FLAGS='-Wno-error -Wno-unknown-warning-option -Wno-unused-command-line-argument' \
    obj/main/betaflight_SITL.elf -j6
) > .tools/betaflight-build.log 2>&1
python3 tools/import_config.py
python3 tools/configure_betaflight.py
printf '\nReady. Double-click Launch Flight Sim.command.\n'
