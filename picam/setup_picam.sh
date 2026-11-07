#!/usr/bin/env bash
# Build Raspberry Pi's libcamera (with Pi 5 PiSP support) + rpicam-apps on Ubuntu 24.04.
# Run ON the Pi:  bash ~/setup_picam.sh        (asks for the sudo password once)
# Takes ~20-40 min on a Pi 5. Safe to re-run: finished steps are skipped.
set -euo pipefail

SRC="$HOME/picam-build"
JOBS=3
mkdir -p "$SRC"
sudo -v

echo "==> [1/5] apt build dependencies"
sudo apt-get update
sudo apt-get install -y git cmake meson ninja-build pkg-config python3-dev python3-pip \
  python3-jinja2 python3-yaml python3-ply python3-venv pybind11-dev \
  libboost-dev libboost-program-options-dev libgnutls28-dev openssl libtiff-dev \
  libglib2.0-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libevent-dev \
  libdrm-dev libjpeg-dev libpng-dev libexif-dev libepoxy-dev libudev-dev \
  libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev nlohmann-json3-dev

echo "==> [2/5] remove Ubuntu's libcamera 0.2 (no Pi 5 support; would shadow ours)"
sudo apt-get remove -y gstreamer1.0-libcamera libcamera-tools libcamera-ipa libcamera0.2 || true

echo "==> [3/5] build libcamera (raspberrypi fork, includes libpisp)"
cd "$SRC"
[ -d libcamera ] || git clone --depth 1 https://github.com/raspberrypi/libcamera.git
cd libcamera
if [ ! -d build ]; then
  meson setup build --buildtype=release \
    -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp \
    -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled \
    -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
fi
ninja -C build -j"$JOBS"
sudo ninja -C build install
sudo ldconfig

echo "==> [4/5] build rpicam-apps"
cd "$SRC"
[ -d rpicam-apps ] || git clone --depth 1 https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps
if [ ! -d build ]; then
  meson setup build -Denable_libav=disabled -Denable_drm=enabled -Denable_egl=disabled \
    -Denable_qt=disabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
fi
ninja -C build -j"$JOBS"
sudo ninja -C build install
sudo ldconfig

echo "==> [5/5] gstreamer plugin path for the new libcamerasrc"
GST_DIR=$(find /usr/local/lib -type d -name gstreamer-1.0 | head -1 || true)
if [ -n "$GST_DIR" ] && ! grep -q GST_PLUGIN_PATH "$HOME/.bashrc"; then
  echo "export GST_PLUGIN_PATH=$GST_DIR:\${GST_PLUGIN_PATH:-}" >> "$HOME/.bashrc"
fi

echo "==> done. Test with:  rpicam-hello --list-cameras"
