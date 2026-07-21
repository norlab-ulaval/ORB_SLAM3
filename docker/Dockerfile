FROM ubuntu:22.04

ARG USERNAME=user
ARG PASSWORD=docker
ARG MY_UID=1000

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo build-essential cmake ninja-build git wget curl unzip ca-certificates pkg-config \
    libopencv-dev libeigen3-dev libboost-serialization-dev libssl-dev python3-dev \
    gdb vim mesa-utils \
    libx11-dev libgl1-mesa-dev libegl1-mesa-dev libepoxy-dev libglew-dev \
    libjpeg-dev libpng-dev libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \
    && rm -rf /var/lib/apt/lists/*

# Pangolin is a system dependency (not part of the bind-mounted repo), built
# into the image so container startup doesn't have to rebuild it every time.
#
# Pinned to v0.6 (not the current v0.9.x): Pangolin v0.7+ rewrote its X11
# windowing backend to require EGL instead of classic GLX. The NVIDIA
# proprietary driver on the host does not support the EGL X11 platform
# (verified: forcing the NVIDIA EGL ICD makes eglInitialize fail for
# EGL_PLATFORM_X11, while GLX works fine), so any v0.7+ Pangolin build fails
# to actually create a visible window even though it reports no error. v0.6
# is the last release using GLX for X11, which this GPU/driver already
# handles correctly.
RUN git clone --branch v0.6 --depth 1 https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin \
    && cmake -S /opt/Pangolin -B /opt/Pangolin/build -GNinja -DCMAKE_BUILD_TYPE=Release -DCPP11_NO_BOOST=1 \
    && cmake --build /opt/Pangolin/build \
    && cmake --install /opt/Pangolin/build \
    && ldconfig \
    && rm -rf /opt/Pangolin

RUN useradd -m -u ${MY_UID} -s /bin/bash ${USERNAME} \
    && echo "${USERNAME}:${PASSWORD}" | chpasswd \
    && usermod -aG sudo ${USERNAME}

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV WORKSPACE=/home/${USERNAME}/code
USER ${USERNAME}
WORKDIR ${WORKSPACE}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
