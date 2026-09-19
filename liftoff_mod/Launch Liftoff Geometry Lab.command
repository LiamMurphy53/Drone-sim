#!/bin/zsh
cd -- "$(dirname -- "$0")" || exit 1
python3 tools/launch_mac.py
result=$?
if (( result != 0 )); then
  read -r '?Press Return to close.'
fi
exit "$result"
