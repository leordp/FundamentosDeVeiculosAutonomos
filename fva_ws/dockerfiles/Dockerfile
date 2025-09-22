FROM ubuntu:22.04

ARG USERNAME=fva
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG WORKSPACE=/home/fva_ws

ENV HOME=/home
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV TZ=Etc/UTC
ENV COPPELIASIM_VERSION=4_10_0
ENV COPPELIASIM_DIR=/opt/CoppeliaSim

# 0. Set shell to use pipefail option
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# 1. Install sudo
RUN mkdir -p ${WORKSPACE} && \
    apt-get update && apt-get install -y sudo \
    && rm -rf /var/lib/apt/lists/*

# 2. Create group & user
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd  --uid  $USER_UID \
             --gid  $USER_GID \
             --create-home \
             --shell /bin/bash \
             $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod -aG sudo $USERNAME

WORKDIR ${WORKSPACE}

# 3. Basic build essentials + X11 + Python + tmux + tmuxinator
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    unzip \
    build-essential \
    git \
    wget \
    curl \
    nano \
    htop \
    ca-certificates \
    libglu1-mesa \
    libxi6 \
    libxrender1 \
    libxext6 \
    libxkbcommon-x11-0 \
    libqt5widgets5 \
    libqt5gui5 \
    libqt5core5a \
    libx11-xcb1 \
    libxcb-xinerama0 \
    xvfb \
    x11-utils \
    socat \
    tmux \
    ruby \
    ruby-dev \
    python3-tk \
    tk \
    # libs extras para o CoppeliaSim
    libzmq5 \
    libsodium23 && \
    rm -rf /var/lib/apt/lists/* && \
    ldconfig && \
    gem install --no-document tmuxinator

# 4. Instalar pacotes Python
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir \
        coppeliasim-zmqremoteapi-client \
        matplotlib \
        numpy \
        opencv-python \
        scipy

# 5. Baixar e instalar CoppeliaSim
RUN mkdir -p ${COPPELIASIM_DIR} && \
    wget -q https://downloads.coppeliarobotics.com/V${COPPELIASIM_VERSION}_rev0/CoppeliaSim_Edu_V${COPPELIASIM_VERSION}_rev0_Ubuntu22_04.tar.xz -O /tmp/coppeliasim.tar.xz && \
    tar -xJf /tmp/coppeliasim.tar.xz -C ${COPPELIASIM_DIR} --strip-components=1 && \
    rm /tmp/coppeliasim.tar.xz

# 6. Expose remote API ports (classic 19997, ZMQ 23000-23050)
EXPOSE 19997 23000-23050

# 7. Config prompt + alias
RUN echo "force_color_prompt=yes" >> $HOME/.bashrc && \
    echo "alias ls='ls --color=auto'" >> $HOME/.bashrc

# 8. Adicionar simulador ao PYTHONPATH
ENV PYTHONPATH="${PYTHONPATH}:/home/fva_ws/src/fundamentos_veiculos_autonomos/simulador"
