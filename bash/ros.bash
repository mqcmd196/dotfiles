#!/bin/bash

# for using ROS2 on macOS
if [ "$(uname)" == 'Darwin' ]; then
    export OPENSSL_ROOT_DIR=/usr/local/opt/openssl@1.1
fi

# source `catkin locate --shell-verbs`
echo "CMAKE_PREFIX_PATH: ""$CMAKE_PREFIX_PATH"
