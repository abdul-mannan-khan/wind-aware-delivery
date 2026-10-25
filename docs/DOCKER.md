# The Docker image

**`abdulmannan617/wind-aware-delivery`** on Docker Hub
(tags: `latest` and the build date, e.g. `2026-10-13`).

| Contents | Where |
|---|---|
| Ubuntu 22.04, Python 3.10, NumPy/SciPy/Matplotlib/pandas (versions of `requirements.txt`), AirSim Python client, Jupyter Lab, ffmpeg, tmux | system |
| This repository | `/root/wind-aware-delivery` (`git pull` to update) |
| AirSim 1.8.1 Neighborhood environment (Unreal Engine 4, Linux build) | `/opt/AirSim/AirSimNH/LinuxNoEditor/AirSimNH.sh` |
| `start_airsim`, `stop_airsim` | `/usr/local/bin` (links to `airsim/*.sh`) |
| User `airsim` (uid 1000) | Unreal Engine refuses to run as root, so AirSim runs as this user |

Size: about 4 GB to download, 8 GB on disk.

## Use on Vast.ai
See [VAST_AI_GUIDE.md](VAST_AI_GUIDE.md). Image path: `abdulmannan617/wind-aware-delivery:v2`.

## Use on your own Linux computer
```bash
docker pull abdulmannan617/wind-aware-delivery:v2
# simulator only (no GPU needed); results stay in ./results on your computer
docker run --rm -it -v "$PWD/results:/root/wind-aware-delivery/results/experiments" \
    abdulmannan617/wind-aware-delivery:v2
# with AirSim: needs an NVIDIA GPU and the NVIDIA Container Toolkit
docker run --rm -it --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all abdulmannan617/wind-aware-delivery:v2
```

## Rebuild the image
```bash
bash docker/build.sh            # build
bash docker/build.sh --push     # build and push (after `docker login`)
```
The Dockerfile copies the packaged AirSimNH environment out of an existing AirSim image (build argument
`AIRSIM_ENV_IMAGE`) and clones this repository from GitHub (build arguments `REPO` and `REF`).
