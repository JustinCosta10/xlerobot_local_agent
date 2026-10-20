#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
colcon build --symlink-install --event-handlers console_direct+ "$@"
