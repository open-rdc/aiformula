
FROM kyo0221/cudagl:12.8.1-devel-ubuntu22.04-torch-humble

ENV USER_NAME=nvidia
ENV MYWORKSPACE_NAME=formula_ws

# create user
RUN useradd -m -s /bin/bash ${USER_NAME} && \
    echo "${USER_NAME}:${USER_NAME}" | chpasswd && \
    adduser ${USER_NAME} sudo && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USER_NAME}
WORKDIR /home/${USER_NAME}
ENV HOME=/home/${USER_NAME}

# install aiformula repository
RUN mkdir ~/$MYWORKSPACE_NAME && \
    cd ~/$MYWORKSPACE_NAME && \
    git clone --recursive -b fix/docker https://github.com/open-rdc/aiformula src

# download YOLOPv2 weights
RUN wget -q https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt \
    -O ~/$MYWORKSPACE_NAME/src/perception/road_detector/data/weights/yolopv2.pt

# config setting
COPY config/.bashrc /home/${USER_NAME}/.bashrc
COPY config/.vimrc /home/${USER_NAME}/.vimrc
COPY config/.tmux.conf /home/${USER_NAME}/.tmux.conf

# install python dependencies
COPY config/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

CMD ["/bin/bash"]
