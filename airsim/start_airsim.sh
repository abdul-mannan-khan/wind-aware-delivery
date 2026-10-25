#!/bin/bash
# Start AirSim (Neighborhood map, headless) inside the wind-aware-delivery container.
# Usage: start_airsim [settings.json]     (default: settings_demo.json next to this script)
# Waits until the AirSim API answers on port 41451 (usually 30-120 s; up to 6 min on slow disks).
HERE=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
SETTINGS=${1:-$HERE/settings_demo.json}
BIN=/opt/AirSim/AirSimNH/LinuxNoEditor/AirSimNH.sh
[ -x "$BIN" ] || { echo "AirSimNH not found at $BIN (are you inside the container?)"; exit 1; }
if (echo > /dev/tcp/127.0.0.1/41451) 2>/dev/null; then echo "AirSim is already running"; exit 0; fi
nvidia-smi -L >/dev/null 2>&1 || echo "WARNING: no NVIDIA GPU visible - AirSim needs a GPU instance"
# Vulkan driver description (normally provided by the NVIDIA container runtime)
if [ ! -e /etc/vulkan/icd.d/nvidia_icd.json ] && [ ! -e /usr/share/vulkan/icd.d/nvidia_icd.json ]; then
  mkdir -p /usr/share/vulkan/icd.d
  echo '{"file_format_version":"1.0.0","ICD":{"library_path":"libGLX_nvidia.so.0","api_version":"1.3.0"}}' \
    > /usr/share/vulkan/icd.d/nvidia_icd.json
fi
mkdir -p /home/airsim/Documents/AirSim
cp "$SETTINGS" /home/airsim/Documents/AirSim/settings.json
chown -R airsim: /home/airsim/Documents
if ! pgrep -f "Xvfb :99" >/dev/null; then
  setsid Xvfb :99 -screen 0 1920x1080x24 +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 </dev/null &
fi
sleep 2
touch /tmp/airsim.log && chmod 666 /tmp/airsim.log
# detached from this terminal, so that closing it does not stop AirSim
setsid runuser -u airsim -- env HOME=/home/airsim DISPLAY=:99 bash -c \
  "cd $(dirname $BIN) && exec ./$(basename $BIN) -RenderOffScreen -nosound" >/tmp/airsim.log 2>&1 </dev/null &
echo -n "starting AirSim "
for i in $(seq 1 120); do
  sleep 3; echo -n "."
  (echo > /dev/tcp/127.0.0.1/41451) 2>/dev/null && { echo; echo "AirSim API is up after ~$((i * 3)) s"; exit 0; }
  pgrep -f "Binaries/Linux/AirSimNH" >/dev/null || [ $i -lt 5 ] || \
    { echo; echo "AirSim stopped. Last lines of /tmp/airsim.log:"; tail -20 /tmp/airsim.log; exit 1; }
done
echo; echo "AirSim did not answer within 6 min. See /tmp/airsim.log"; exit 1
