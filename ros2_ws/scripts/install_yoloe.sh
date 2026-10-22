#!/usr/bin/env bash
# Isolated inference environment; leaves ROS/Isaac's Python dependencies alone.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
mkdir -p .deps/models .deps/yoloe-config
touch .deps/COLCON_IGNORE
# Reuse the system pip; this also works on minimal Ubuntu without ensurepip.
python3 -m venv --without-pip --system-site-packages .deps/yoloe
.deps/yoloe/bin/python -m pip install --no-cache-dir \
    torch==2.9.1 torchvision==0.24.1 --index-url https://download.pytorch.org/whl/cu128
# NumPy 1.x also works with Jazzy's system cv_bridge/OpenCV binaries.
.deps/yoloe/bin/python -m pip install --no-cache-dir \
    numpy==1.26.4 opencv-python==4.11.0.86 ultralytics==8.4.184
export YOLO_CONFIG_DIR="$PWD/.deps/yoloe-config"
export YOLO_AUTOINSTALL=false
.deps/yoloe/bin/python - <<'PY'
from pathlib import Path
from ultralytics import YOLOE
model = Path('.deps/models/yoloe-11s-seg-pf.pt').resolve()
YOLOE(str(model))
print(f'YOLOE ready: {model}')
PY
echo 'Source scripts/env.sh again, then run scripts/build.sh.'
