#!/bin/bash

# for using ROS2 on macOS
if [ "$(uname)" == 'Darwin' ]; then
    export OPENSSL_ROOT_DIR=/usr/local/opt/openssl@1.1
fi

# catkin_add_configs(){
#     catkin config --profile default_export --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# }

ros_workspace_set(){
    ROS_WORKSPACE_DIR=$1
    ROS_WORKSPACE_DEVEL_SETUP=${ROS_WORKSPACE_DIR}/devel/setup.bash
    if [ -e $ROS_WORKSPACE_DEVEL_SETUP ]; then
        ROS_WORKSPACE_DISPLAY=`basename ${ROS_WORKSPACE_DIR}`
        PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]$(__git_ps1) \[\033[01;32m\](ROS_workspace:${ROS_WORKSPACE_DISPLAY}) \[\033[00m\]$ '
        source $ROS_WORKSPACE_DEVEL_SETUP
    else
        echo No such ROS workspace
    fi
}

catkin_before_build(){
    local catkin_ws=$(echo $CMAKE_PREFIX_PATH | cut -d: -f1)/..
    (cd "${catkin_ws}" && catkin config | grep -- -DCMAKE_EXPORT_COMPILE_COMMANDS=ON >/dev/null)
    local catkin_config_contains_compile_commands=$?
    if [ $catkin_config_contains_compile_commands -ne 0 ]; then
        echo catkin config does not include -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
        (
            cd "${catkin_ws}" &&
                catkin config -a --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
        )
    else
        echo catkin config already includes -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
        cd "${catkin_ws}" && catkin config
    fi
}

catkin_generate_compile_commands_json(){
    echo Generating workspace compile_commands.json...
    local catkin_ws=$(echo $CMAKE_PREFIX_PATH | cut -d: -f1)/..
    cd ${catkin_ws}
    if [ -e compile_commands.json ]; then
        rm compile_commands.json
    fi
    concatenated="compile_commands.json"
    echo "[" > $concatenated
    first=1
    for d in build/*
    do
        f="$d/compile_commands.json"
        if test -f "$f"; then
            if [ $first -eq 0 ]; then
                echo "," >> $concatenated
            fi
            cat $f | sed '1d;$d' >> $concatenated
        fi
        first=0
    done
    echo "]" >> $concatenated
}

catkin_appropriate_build(){
    local catkin_ws=$(echo $CMAKE_PREFIX_PATH | cut -d: -f1)/..
    cd $catkin_ws
    catkin_before_build
    local cpu_cores=$(python -c 'import multiprocessing as m; print(m.cpu_count());')
    local jobs=$(($cpu_cores / 2))
    catkin b -j$jobs -p$jobs -c
    catkin_generate_compile_commands_json
    ros_workspace_set $catkin_ws
}

