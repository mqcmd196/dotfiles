FROM ubuntu:noble

ENV DEBIAN_FRONTEND=noninteractive

# For manage user
RUN apt update -qq && \
    apt upgrade -y -qq && \
    apt install -y -qq --no-install-recommends sudo software-properties-common

# install prerequisites
RUN apt install -y -qq --no-install-recommends \
    git \
    ansible

# Add non root user
ARG USERNAME=user
ARG PASSWORD=user
ARG GROUPNAME=user
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID $GROUPNAME && \
    useradd -m -s /bin/bash -u $UID -g $GID -G sudo $USERNAME && \
    echo $USERNAME:$PASSWORD | chpasswd && \
    echo "$USERNAME   ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

WORKDIR /home/$USERNAME/dotfiles
COPY ./ .
RUN ./test/docker/run_deb