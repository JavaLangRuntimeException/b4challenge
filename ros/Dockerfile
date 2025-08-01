FROM ubuntu:22.04

# ビルド時に渡すパラメータ
ARG UID
ARG GID
ARG USER_NAME
ARG GROUP_NAME

# Timezone, Launguage設定
RUN apt update \
    && apt install -y --no-install-recommends \
        locales sudo \
    software-properties-common tzdata \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe

ENV LANG=en_US.UTF-8
ENV TZ=Asia/Tokyo

# ライブラリの追加
RUN apt update \
    && apt install -y --no-install-recommends \
        curl git tmux gnupg2 lsb-release python3-pip vim wget build-essential ca-certificates \
        x11-apps x11-utils mesa-utils

# ROS2のインストール
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update \
    && apt upgrade -y\
    && DEBIAN_FRONTEND=noninteractive \
    && apt install -y --no-install-recommends \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    '~nros-humble-rqt*'

# ユーザとグループの追加
# GIDが既に存在する場合は既存のグループを使用、存在しない場合のみグループを作成
RUN if getent group $GID >/dev/null 2>&1; then \
        echo "GID $GID already exists, using existing group"; \
    else \
        groupadd -g $GID $GROUP_NAME; \
        echo "Created new group $GROUP_NAME with GID $GID"; \
    fi

# ユーザーを作成（UIDが既に存在する場合の処理も含む）
RUN if getent passwd $UID >/dev/null 2>&1; then \
        echo "UID $UID already exists, using existing user"; \
        EXISTING_USER=$(getent passwd $UID | cut -d: -f1); \
        usermod -l $USER_NAME -d /home/$USER_NAME -m $EXISTING_USER 2>/dev/null || true; \
    else \
        useradd -u $UID -g $GID -s /bin/bash -m $USER_NAME; \
        echo "Created new user $USER_NAME with UID $UID"; \
    fi
RUN usermod -a -G video $USER_NAME
RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers && chmod 0440 /etc/sudoers && chmod g+w /etc/passwd

# ROS2環境設定
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USER_NAME/.bashrc \
    && echo "source /home/$USER_NAME/ws/install/setup.bash" >> /home/$USER_NAME/.bashrc

USER $USER_NAME

WORKDIR /home/${USER_NAME}/ws
RUN mkdir -p src

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/humble/setup.bash \
    && sudo apt-get update \
    && sudo rosdep init \
    && rosdep update \
    && rosdep install --from-paths src --rosdistro humble -i -y

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USER_NAME}/.bashrc
RUN echo "source ~/ws/install/setup.bash" >> /home/${USER_NAME}/.bashrc

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --parallel-workers 1 --event-handlers console_cohesion+ console_direct+ log+

RUN sudo rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
