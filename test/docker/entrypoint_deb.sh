#!/bin/bash

sudo apt update && apt upgrade -y
sudo apt install -y --no-install-recommends \
    git \
    python3 \
    python3-pip \
    snapd \
    && rm -rf /var/lib/apt/lists/*
sudo pip3 install pyyaml distro

./setup
./test/test
