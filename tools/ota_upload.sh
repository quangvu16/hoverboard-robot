#!/usr/bin/env bash
# Build an ESP32 sketch and flash it over Wi-Fi (firmware ota.h).
#
#   tools/ota_upload.sh [sketch-dir] [host]
#
#   sketch-dir  default firmware/ldrobot_ld14p_esp32_mpu9250_ros
#               (a name containing "esp32s3" builds for the ESP32-S3)
#   host        default hoverboard.local; an IP address works too
#
# The password is OTA_PASSWORD in the sketch's secrets.h. It is compiled into
# the new firmware, so it is required there: without it the new image would have
# OTA off and the next update would need the USB cable again. $OTA_PASSWORD
# only overrides the password used to log in to the board now (when the board
# still runs an older one).
# The robot must stand still: the board ignores update requests while it moves.
# The board connects back to this PC on TCP $OTA_HOST_PORT (default 38232);
# with ufw on:  sudo ufw allow 38232/tcp
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SKETCH="$(cd "${1:-$ROOT/firmware/ldrobot_ld14p_esp32_mpu9250_ros}" && pwd)"
HOST="${2:-hoverboard.local}"
HOST_PORT="${OTA_HOST_PORT:-38232}"
NAME="$(basename "$SKETCH")"

case "$NAME" in
  *esp32s3*) FQBN="esp32:esp32:esp32s3" ;;
  *)         FQBN="esp32:esp32:esp32" ;;
esac

NEW_PASS=""
if [[ -f "$SKETCH/secrets.h" ]]; then
  NEW_PASS="$(sed -n 's/^[[:space:]]*#define[[:space:]]\+OTA_PASSWORD[[:space:]]\+"\(.*\)".*/\1/p' "$SKETCH/secrets.h" | head -n1)"
fi
if [[ -z "$NEW_PASS" ]]; then
  echo "No OTA_PASSWORD in $SKETCH/secrets.h: add #define OTA_PASSWORD \"...\" there." >&2
  echo "(The board must also run firmware with a password, flashed once over USB.)" >&2
  exit 1
fi
PASS="${OTA_PASSWORD:-$NEW_PASS}"

ESPOTA="$(ls -1 "$HOME"/.arduino15/packages/esp32/hardware/esp32/*/tools/espota.py 2>/dev/null | sort -V | tail -n1)"
[[ -n "$ESPOTA" ]] || { echo "espota.py not found: install the esp32 core (arduino-cli core install esp32:esp32)" >&2; exit 1; }

# espota.py does not resolve .local names on every system, so resolve here.
if [[ "$HOST" =~ ^[0-9]+(\.[0-9]+){3}$ ]]; then
  IP="$HOST"
else
  IP="$(getent ahostsv4 "$HOST" 2>/dev/null | awk 'NR==1{print $1}')"
  [[ -n "$IP" ]] || { echo "Cannot resolve $HOST: use the board's IP (printed on the Serial Monitor at boot)" >&2; exit 1; }
fi

BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT

echo "== build $NAME for $FQBN"
arduino-cli compile --fqbn "$FQBN" --output-dir "$BUILD" "$SKETCH"
BIN="$BUILD/$NAME.ino.bin"
[[ -f "$BIN" ]] || { echo "build produced no $BIN" >&2; exit 1; }

echo "== upload to $HOST ($IP), $(stat -c %s "$BIN") bytes"
echo "   the wheels are held at zero until the board reboots"
python3 "$ESPOTA" -i "$IP" -p 3232 -P "$HOST_PORT" -a "$PASS" -f "$BIN" -r
echo "== done: the board reboots into the new firmware"
