#!/usr/bin/env bash

source /os/os-release

echo "Sysdeps are
     fonts-powerline
"

if [ $ID == "ubuntu" ] && [ $VERSION_CODENAME == "bionic"]; then
    apt install -y fonts-powerline
fi
