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

WORKDIR /home/$USERNAME/dotfiles
COPY ./ .
RUN ./test/docker/run_deb