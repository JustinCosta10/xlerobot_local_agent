#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
# The extracted .deps/usr prefix contains Python, but ROS build modules belong
# to the system interpreter. Pin it so fresh CMake discovery uses those modules.
colcon build --symlink-install --event-handlers console_direct+ \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
