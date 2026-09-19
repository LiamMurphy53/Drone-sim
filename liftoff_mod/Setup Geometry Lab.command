#!/bin/zsh
cd -- "$(dirname -- "$0")" || exit 1
python3 tools/setup_mac.py
result=$?
read -r '?Press Return to close.'
exit "$result"
