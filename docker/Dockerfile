# Quadrotor package delivery through urban wind corridors - simulation image.
#   * Python simulator (6-DOF quadrotor, urban wind model, planners) with all dependencies
#   * AirSim 1.8.1 "Neighborhood" environment (AirSimNH, Unreal Engine 4, headless) for the AirSim check
#   * this repository, cloned into /root/wind-aware-delivery
# Build:  bash docker/build.sh      (see docs/DOCKER.md)
ARG AIRSIM_ENV_IMAGE=abdulmannan617/sar-airsim:latest

# Stage 1: only the packaged AirSimNH environment is taken from this image (Microsoft AirSim release binary)
FROM ${AIRSIM_ENV_IMAGE} AS airsim_src
FROM ubuntu:22.04 AS airsim_env
COPY --from=airsim_src /opt/AirSim/AirSimNH /opt/AirSim/AirSimNH
# debug symbols (2.1 GB) are not needed to run the environment
RUN rm -f /opt/AirSim/AirSimNH/LinuxNoEditor/AirSimNH/Binaries/Linux/AirSimNH.debug \
          /opt/AirSim/AirSimNH/LinuxNoEditor/AirSimNH/Binaries/Linux/AirSimNH.sym

FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 \
    NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all DISPLAY=:99 PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-dev build-essential git ca-certificates curl wget nano vim less tmux htop \
        rsync zip unzip openssh-server ffmpeg fonts-dejavu-core \
        xvfb libvulkan1 libgl1 libglu1-mesa libegl1 libxrandr2 libxinerama1 libxcursor1 libxi6 libxext6 \
        libxrender1 libxss1 libasound2 libnss3 libsm6 libice6 libpulse0 \
    && rm -rf /var/lib/apt/lists/*

# Python packages (same versions as the paper results) + Jupyter for the Vast.ai browser terminal
COPY requirements.txt /tmp/requirements.txt
RUN grep -v -E "^(airsim|msgpack)" /tmp/requirements.txt > /tmp/req_sim.txt \
    && pip3 install --no-cache-dir -r /tmp/req_sim.txt jupyterlab \
    && python3 -c "import numpy, scipy, matplotlib, pandas, yaml; print('python ok')"
# The AirSim client needs tornado 4 (Jupyter needs tornado 6): it lives in its own folder, /opt/airsim_deps,
# and the AirSim scripts import it through sim/airsim_compat.py. (Its setup.py imports msgpackrpc, hence the order.)
RUN pip3 install --no-cache-dir --target /opt/airsim_deps tornado==4.5.3 msgpack-python==0.5.6 msgpack-rpc-python==0.4.1 \
    && PYTHONPATH=/opt/airsim_deps pip3 install --no-cache-dir --no-deps --no-build-isolation --target /opt/airsim_deps airsim==1.8.1 \
    && PYTHONPATH=/opt/airsim_deps python3 -c "import airsim; print('airsim client ok')"

# Unreal Engine refuses to run as root: AirSim runs as user 'airsim' (uid 1000)
RUN useradd -m -u 1000 -s /bin/bash airsim && usermod -aG video airsim
COPY --from=airsim_env --chown=airsim:airsim /opt/AirSim/AirSimNH /opt/AirSim/AirSimNH

# The repository (public); `git pull` inside the container updates it
ARG REPO=https://github.com/abdul-mannan-khan/wind-aware-delivery.git
ARG REF=main
ARG REPO_COMMIT=unknown
RUN echo "repository at ${REPO_COMMIT}" && git clone --branch ${REF} ${REPO} /root/wind-aware-delivery \
    && ln -s /root/wind-aware-delivery/airsim/start_airsim.sh /usr/local/bin/start_airsim \
    && ln -s /root/wind-aware-delivery/airsim/stop_airsim.sh /usr/local/bin/stop_airsim \
    && cp /root/wind-aware-delivery/docker/welcome.sh /etc/profile.d/zz-welcome.sh \
    && echo '[ -n "$PS1" ] && bash /etc/profile.d/zz-welcome.sh' >> /root/.bashrc
WORKDIR /root/wind-aware-delivery
CMD ["bash"]
