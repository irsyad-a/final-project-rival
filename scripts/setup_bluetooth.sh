#!/usr/bin/env bash
set -e

# Setup /dev/rfcomm0 for HC-05/HC-06
# Cara pakai:
#   1) Interaktif (tanpa argumen): ./setup_bluetooth.sh
#      → Skrip akan menanyakan MAC dan channel (default 1)
#   2) Langsung: ./setup_bluetooth.sh 98:D3:31:F9:12:34 1
#   3) ENV: HC05_MAC dan HC05_CH (opsional)
#
# Contoh MAC: 00:25:00:00:C3:44

MAC="${1:-}"
CHANNEL_DEFAULT="1"
CHANNEL="${2:-}"

# Jika tak ada argumen, coba ENV
if [ -z "$MAC" ] && [ -n "$HC05_MAC" ]; then
  MAC="$HC05_MAC"
fi
if [ -z "$CHANNEL" ] && [ -n "$HC05_CH" ]; then
  CHANNEL="$HC05_CH"
fi

# Prompt interaktif bila masih kosong
if [ -z "$MAC" ]; then
  echo "Tidak ada MAC yang diberikan."
  read -rp "Masukkan BT MAC HC-05 (contoh 00:25:00:00:C3:44): " MAC
fi
if [ -z "$MAC" ]; then
  echo "Error: MAC kosong. Batal."
  exit 1
fi
if [ -z "$CHANNEL" ]; then
  read -rp "Masukkan channel [${CHANNEL_DEFAULT}]: " CHANNEL
  CHANNEL="${CHANNEL:-$CHANNEL_DEFAULT}"
fi

echo "Binding /dev/rfcomm0 ke $MAC channel $CHANNEL ..."
sudo rfcomm release 0 || true
if ! sudo rfcomm bind 0 "$MAC" "$CHANNEL"; then
  echo "Gagal bind. Coba lepas device lain yang memakai rfcomm0 (screen/lain), lalu ulangi."
  exit 1
fi
echo "Selesai. Device: /dev/rfcomm0"
if [ -e /dev/rfcomm0 ]; then
  ls -l /dev/rfcomm0 || true
else
  echo "Peringatan: /dev/rfcomm0 belum muncul. Coba ulang: sudo rfcomm release 0 && sudo rfcomm bind 0 \"$MAC\" \"$CHANNEL\""
fi

