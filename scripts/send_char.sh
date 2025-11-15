#!/usr/bin/env bash
set -e

# Usage:
#   ./scripts/send_char.sh /dev/rfcomm0 w [baud]
# Default baud: 9600 (common for HC-05 in data mode)

if [ -z "$1" ] || [ -z "$2" ]; then
  echo "Usage: $0 <device> <char> [baud]"
  echo "Example: $0 /dev/rfcomm0 w 9600"
  exit 1
fi

DEV="$1"
CHAR="$2"
BAUD="${3:-9600}"

if [ ${#CHAR} -ne 1 ]; then
  echo "Error: <char> must be a single character (e.g., w, a, s, d, x)"
  exit 1
fi

if [ ! -e "$DEV" ]; then
  echo "Error: device $DEV not found. Bind with ./scripts/setup_bluetooth.sh <MAC> 1"
  exit 1
fi

echo "Configuring $DEV @ ${BAUD} baud (raw)..."
sudo stty -F "$DEV" "$BAUD" raw -echo -onlcr -ocrnl -opost -isig -icanon min 0 time 0 || true

echo -n "Sending '$CHAR' to $DEV ..."
printf "%b" "$CHAR" | sudo tee "$DEV" >/dev/null
echo " done."


