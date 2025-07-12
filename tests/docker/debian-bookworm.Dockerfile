FROM debian:bookworm

ENV DEBIAN_FRONTEND=noninteractive

# For manage user
RUN apt update -qq && \
    apt upgrade -y -qq && \
    apt install -y -qq --no-install-recommends \
    ansible apt git software-properties-common sudo

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
COPY non-sudoer ./non-sudoer
COPY roles ./roles
COPY setup_sudoer.yml .
RUN ansible-playbook setup_sudoer.yml -K
COPY tests ./tests
RUN ./tests/test_emacs.sh
RUN ./tests/test_shell.sh
