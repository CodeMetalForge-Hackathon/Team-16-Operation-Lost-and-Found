#!/usr/bin/env bash
# Run ON the Pi (no sudo). Builds the Python bindings for the HailoRT 4.24.0 that is already
# installed in /usr/local, and downloads a pretrained person-detection model (YOLOv8s .hef).
# Everything goes into ~/team16_thermal; nothing outside that folder is touched.
#   bash setup_hailo_py.sh
# Then run thermal_pi.py with:  PYTHONPATH=$HOME/team16_thermal/hailort/hailort/libhailort/bindings/python/platform
set -euo pipefail

BASE="$HOME/team16_thermal"
mkdir -p "$BASE/models"
cd "$BASE"

[ -d venv ] || python3 -m venv --system-site-packages venv
. venv/bin/activate
pip install -q pybind11 setuptools wheel

[ -d hailort ] || git clone -q --depth 1 --branch v4.24.0 https://github.com/hailo-ai/hailort.git

SRC="$BASE/hailort/hailort/libhailort/bindings/python/src"
cmake -S "$SRC" -B pybuild -DCMAKE_BUILD_TYPE=Release -DPYBIND11_PYTHON_VERSION=3.12 \
  -DLIBHAILORT_PATH=/usr/local/lib/libhailort.so.4.24.0 -DHAILORT_INCLUDE_DIR=/usr/local/include
nice -n 19 cmake --build pybuild -j2
cmake --install pybuild

if [ ! -s models/yolov8s.hef ]; then
  python3 - <<'EOF'
import urllib.request
u = "https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8/yolov8s.hef"
urllib.request.urlretrieve(u, "models/yolov8s.hef")
EOF
fi
ls -l models/yolov8s.hef
echo "done"
