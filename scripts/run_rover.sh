  #!/usr/bin/env bash
set -e

# Build (if needed) and run autonomous_rover.
# Optional arg: camera URL override
#
# Examples:
#   ./scripts/run_rover.sh
#   ./scripts/run_rover.sh http://192.168.1.100:8080/video

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

if [ ! -f Makefile ]; then
  cmake ..
fi

make autonomous_rover -j$(nproc)

CAM_URL="${1:-}"
if [ -z "$CAM_URL" ]; then
  ./autonomous_rover
else
  ./autonomous_rover "$CAM_URL"
fi


