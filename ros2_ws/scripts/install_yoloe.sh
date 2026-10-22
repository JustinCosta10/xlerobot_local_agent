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
    numpy==1.26.4 opencv-python==4.11.0.86 ultralytics==8.4.184 \
    'clip @ git+https://github.com/ultralytics/CLIP.git@7ffa84b3bfa40c42ecc1c77147a855e69cb2dd40'
export YOLO_CONFIG_DIR="$PWD/.deps/yoloe-config"
export YOLO_AUTOINSTALL=false
.deps/yoloe/bin/python - <<'PY'
from pathlib import Path
from ultralytics import YOLOE
from ultralytics.utils.downloads import attempt_download_asset
model = Path('.deps/models/yoloe-11s-seg.pt').resolve()
YOLOE(str(model))
encoder = Path('.deps/models/mobileclip_blt.ts').resolve()
attempt_download_asset(str(encoder))
print(f'YOLOE ready: {model}')
print(f'Text encoder ready: {encoder}')
PY
echo 'Source scripts/env.sh again, then run scripts/build.sh.'
