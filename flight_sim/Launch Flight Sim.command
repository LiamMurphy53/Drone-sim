#!/bin/zsh
cd "${0:A:h}"
/usr/bin/python3 tools/launch.py
if [[ $? -ne 0 ]]; then
  echo "Simulator could not start. See the message above. Press Return to close."
  read
fi
