FROM ubuntu:22.04

ARG USERNAME=fva
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG WORKSPACE=/home/fva_ws

ENV HOME=/home
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV TZ=Etc/UTC
ENV COPPELIASIM_VERSION=4.10.0
ENV COPPELIASIM_DIR=/opt/CoppeliaSim

# 0. Set shell to use pipefail option
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# 1. Install sudo (not present in all minimal images)
RUN mkdir -p ${WORKSPACE} && \
    apt-get update && apt-get install -y sudo \
    && rm -rf /var/lib/apt/lists/*    # keep image slim

# # 2. Create group & user, give a home dir and bash shell
# RUN groupadd --gid $USER_GID $USERNAME && \
#     useradd  --uid  $USER_UID \
#             --gid  $USER_GID \
#             --create-home \
#             --shell /bin/bash \
#             $USERNAME && \
#     # 3. Allow the new user to run any command via sudo with no password
#     echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
#     chmod 0440 /etc/sudoers.d/$USERNAME && \
#     usermod -aG sudo $USERNAME

WORKDIR ${WORKSPACE}
COPY requirements.txt .

# 4. Basic build essentials + X11 + Python
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-venv \
    python3-pip \
    python3-dev \
    sudo \
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
    # Install these packages so that CoppeliaSim doesn't show errors
    libzmq5 \
    libsodium23 && \
    rm -rf /var/lib/apt/lists/* && \
    # refresh linker cache after installing libzmq5 libsodium23 libraries
    ldconfig

# 5. Install Python packages in a virtual environment
RUN echo '#!/bin/bash' >> /usr/local/bin/python && \
    echo 'exec $HOME/venv/bin/python "$@"'  >> /usr/local/bin/python && \
    chmod +x /usr/local/bin/python && \
    ln -sf /usr/local/bin/python /usr/local/bin/python3 && \
    echo '#!/bin/bash' >> /usr/local/bin/ipython && \
    echo 'exec $HOME/venv/bin/python -m IPython "$@"' >> /usr/local/bin/ipython && \
    chmod +x /usr/local/bin/ipython && \
    ln -sf /usr/local/bin/ipython /usr/local/bin/ipython3 && \
    ln -sf $HOME/venv/bin/python /usr/local/bin/python && \
    ln -sf $HOME/venv/bin/python /usr/local/bin/python3 && \
    python3 -m venv $HOME/venv && \
    # Install these packages in the default python3 so Coppelia doesn't show errors
    python3 -m pip install --no-cache-dir pyzmq cbor2 && \
    # Upgrade pip and install requirements in venv
    $HOME/venv/bin/pip install --upgrade pip && \
    $HOME/venv/bin/pip install --no-cache-dir -r requirements.txt

# 6. Copy CoppeliaSim
COPY data/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04.tar.xz /tmp/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04.tar.xz


# 8. Extract CoppeliaSim and Comau files
RUN mkdir -p ${COPPELIASIM_DIR} && \
mkdir -p $HOME/data && \
tar -xJf /tmp/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04.tar.xz -C ${COPPELIASIM_DIR} --strip-components=1 && \
rm /tmp/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04.tar.xz

# 9. Expose remote API ports (classic 19997, ZMQ 23000-23050 for multi-sim)
EXPOSE 19997 23000-23050

# 10. Set up colored prompt and ls alias for the user
RUN echo "force_color_prompt=yes" >> $HOME/.bashrc && \
echo "alias ls='ls --color=auto'" >> $HOME/.bashrc && \
echo "source $HOME/venv/bin/activate" >> $HOME/.bashrc
# echo "export LD_LIBRARY_PATH" >> $HOME/.bashrc && \
# echo "export PKG_CONFIG_PATH" >> $HOME/.bashrc

# 11. Copy the command script to the home directory
COPY start_coppeliasim.sh ${HOME}

# # 12. Create the restart_coppelia.sh script
# RUN echo '#!/bin/bash' > /home/comau/restart_coppelia.sh && \
#     echo 'pkill -f CoppeliaSim || true' >> /home/comau/restart_coppelia.sh && \
#     echo '/home/comau/command.sh &' >> /home/comau/restart_coppelia.sh && \
#     chmod +x /home/comau/restart_coppelia.sh && \
#     chown comau:comau /home/comau/restart_coppelia.sh

# 12. Add alias to .bashrc
RUN echo "alias coppeliasim='/home/start_coppeliasim.sh'" >> /home/fva_ws/.bashrc
