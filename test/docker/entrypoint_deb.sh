#!/bin/bash

sudo apt update && sudo apt upgrade -y
sudo apt install -y --no-install-recommends \
    git \
    python3 \
    python3-apt \
    python3-pip \
    python3-distro \
    snapd \
    && sudo rm -rf /var/lib/apt/lists/*
sudo pip3 install pyyaml

./setup
./test/test
