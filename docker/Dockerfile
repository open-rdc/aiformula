# cudaやcuDNNを利用したい場合は公式サイトを参考に以下のように変更してください
# 例 FROM nvidia/cuda:12.2.0-devel-ubuntu22.04
FROM ubuntu:22.04
# maintainer
LABEL maintainer="Kenta Ishizeki<kentai5420@gmail.com>"
# ROS_DIDTRO
ENV ROS_DISTRO=humble
# username 自分の好きな名前に変更
ENV USER_NAME=nvidia
ENV DEBIAN_FRONTEND=noninteractive
# workspace 好きな名前に変更
ENV MYWORKSPACE_NAME=formula_ws
# basic packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        wget \
        curl \
        git \
        build-essential \
        vim \
        sudo \
        lsb-release \
        locales \
        bash-completion \
        tzdata \
        python3-pip \
        python3-dev \
        software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# locale
RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# create user
RUN useradd -m -s /bin/bash ${USER_NAME} && \
    echo "${USER_NAME}:${USER_NAME}" | chpasswd && \
    adduser ${USER_NAME} sudo && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USER_NAME}
WORKDIR /home/${USER_NAME}
ENV HOME=/home/${USER_NAME}

# ROS2 Humble install
RUN sudo apt-get update && \
    sudo apt-get install -y software-properties-common && \
    sudo add-apt-repository universe && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Other install
RUN sudo apt-get update && \
    sudo apt-get install -y \
        ros-humble-desktop \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        python3-argcomplete \
        ros-dev-tools \
        ros-humble-joint-state-publisher \
        ros-humble-rqt-* \
        ros-humble-xacro \
        ros-$ROS_DISTRO-ros-gz-sim \
        ros-$ROS_DISTRO-ros-gz-bridge

# rosdep
RUN sudo rosdep init || true
RUN rosdep update

# python packages
RUN pip3 install --upgrade pip
RUN pip3 install pyserial

# Layout of terminal
RUN echo "PS1='\[\e[1;31m\]\u@\h\[\e[00m\]:\[\e[1;34m\]\w\[\e[00m\]\$ '" >> ~/.bashrc

# ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ~/${MYWORKSPACE_NAME}/install/setup.bash" >> ~/.bashrc

RUN mkdir ~/$MYWORKSPACE_NAME && \
    cd ~/$MYWORKSPACE_NAME && \
    git clone --recursive -b deploy https://github.com/open-rdc/aiformula src

CMD ["/bin/bash"]
