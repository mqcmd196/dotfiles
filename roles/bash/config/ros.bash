#!/bin/bash

# for using ROS2 on macOS
if [ "$(uname)" == 'Darwin' ]; then
    export OPENSSL_ROOT_DIR=/usr/local/opt/openssl@1.1
fi

init_ros_bash(){
    local cpu_cores=$(python3 -c 'import multiprocessing as m; print(m.cpu_count());')
    local jobs=$(($cpu_cores / 2))
    alias catkinb="catkin b -j$jobs -p$jobs -c"
    alias catkinbt="catkin bt -j$jobs -p$jobs -c"
    if [ -t 1 ]; then
        bind -x '"\C-\M-r": _ross'
    fi
    export ROSCONSOLE_FORMAT='[${severity}][${time}][${node}:${logger}]: ${message}'
}

ros_workspace_init(){
    if [ -e "/opt/ros/melodic/setup.bash" ]; then
        source /opt/ros/melodic/setup.bash # melodic only
    elif [ -e "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
    fi
    catkin config -a --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

ros_workspace_set(){
    ROS_WORKSPACE_DIR=$1
    ROS_WORKSPACE_DEVEL_SETUP=${ROS_WORKSPACE_DIR}/devel/setup.bash
    if [ -e $ROS_WORKSPACE_DEVEL_SETUP ]; then
        source $ROS_WORKSPACE_DEVEL_SETUP
        local catkin_ws=$(cd $(echo $CMAKE_PREFIX_PATH | cut -d: -f1)/.. && pwd)
        ROS_WORKSPACE_DISPLAY=$(basename $catkin_ws)
        echo sourced $ROS_WORKSPACE_DISPLAY
    else
        echo No such ROS workspace
    fi
}

ros_workspace_show(){
    local catkin_ws=$(echo $CMAKE_PREFIX_PATH | cut -d: -f1)/..
    cd $catkin_ws && pwd
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
    local old_dir=$(pwd)
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
    cd $old_dir
}

catkin_after_build(){
    catkin_generate_compile_commands_json
    local catkin_ws=$(echo $CMAKE_PREFIX_PATH | cut -d: -f1)/..
    touch $catkin_ws/.ccls-root
    ros_workspace_set $catkin_ws
}

_ross() {
    local mode
    local var
    local com
    mode=$(echo -e "node\ntopic" | peco)
    if [ ${mode} ]; then
        if [ ${mode} = "node" ]; then
            local var=$(rosnode list | peco)
            local com=$(echo -e "ping\ninfo\nmachine\nkill\ncleanup" | peco)
        elif [ ${mode} = "topic" ]; then
            local var=$(rostopic list | peco)
            local com=$(echo -e "bw\ndelay\necho\nhz\ninfo\npub\ntype\n" | peco)
        fi
    fi
    if [ ${var} ] && [ ${com} ]; then
        local fullcmd="ros${mode} ${com} ${var}"
        READLINE_LINE="$fullcmd"
        READLINE_POINT=${#fullcmd}
    fi
}

if [ -d /opt/ros ]; then
    init_ros_bash
fi

