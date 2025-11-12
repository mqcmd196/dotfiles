#!/usr/bin/env zsh

init_ros_zsh(){
    local cpu_cores=$(python3 -c 'import multiprocessing as m; print(m.cpu_count());')
    local jobs=$(($cpu_cores / 2))
    alias catkinb="catkin b -j$jobs -p$jobs -c -s"
    alias catkinbt="catkin bt -j$jobs -p$jobs -c -s"
    zle -N rospeco
    bindkey '^[^r' rospeco
    ros_console_set_default > /dev/null
}

ros_console_set_debug(){
    echo "Set debug style ROS CONSOLE"
    export ROSCONSOLE_FORMAT='[${severity}][${time}][${node}:${logger}][${file}:L${line}]: ${message}' # ROS 1
    export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}][{time}][{name}][{file_name}:{function_name}:L{line_number}]: {message}' # ROS2
}

ros_console_set_default(){
    echo "Set default ROS CONSOLE"
    export ROSCONSOLE_FORMAT='[${severity}][${time}][${node}:${logger}]: ${message}' # ROS1
    export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}][{time}][{name}]: {message}' # ROS2
    export RCUTILS_COLORIZED_OUTPUT=1
}

ros_workspace_init(){
    source /opt/ros/**/setup.zsh
    if [ -e "/opt/ros/noetic/setup.zsh" ]; then
        source /opt/ros/noetic/setup.zsh
    elif [ -e "/opt/ros/one/setup.zsh" ]; then
        source /opt/ros/one/setup.zsh
    fi
    catkin config -a --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

ros_workspace_basename(){
    local workspaces workspace_name
    if [[ -n $CMAKE_PREFIX_PATH ]]; then
        # Split CMAKE_PREFIX_PATH by ':' and iterate over each path
        workspaces=("${(s/:/)CMAKE_PREFIX_PATH}")
        # Iterate over workspaces to find a .catkin file indicating a ROS workspace
        for workspace in "${workspaces[@]}"; do
            if [[ -f $workspace/.catkin ]]; then
                # Get the directory containing the 'devel' directory, which is the workspace root
                workspace_name="${workspace:h}"
                workspace_name="${workspace_name:t}"
                echo $workspace_name
                return 0
            fi
        done
        echo "No active ROS workspace found."
        return 1
    else
        echo "CMAKE_PREFIX_PATH is not set."
        return 1
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
}

rospeco() {
    # mode=$(echo -e "node\ntopic\nservice\nmsg" | peco --prompt ROS\?\>)
    if [[ $ROS_VERSION = 1 ]]; then
        IFS=$'\n'
        local nodes=($(rosnode list))
        local topics=($(rostopic list))
        local services=($(rosservice list))
        local params=($(rosparam list))
        for i in $nodes; do
            elements=${elements}"$i (node)\n"
        done
        for i in $topics; do
            elements=${elements}"$i (topic)\n"
        done
        for i in $services; do
            elements=${elements}"$i (service)\n"
        done
        for i in $nodes; do
            elements=${elements}"$i (param)\n"
        done
        local select=$(echo -e "${elements}" | peco --prompt ROS\?\>)
        local mode=$(echo ${select} | awk '{print $2}' | sed "s/(//" | sed "s/)//")
        local var=$(echo ${select} | awk '{print $1}')
        if [ ${mode} = "node" ]; then
            local com=$(echo -e "ping\ninfo\nmachine\nkill\ncleanup" | peco --prompt COMMAND\?\>)
        elif [ ${mode} = "topic" ]; then
            local com=$(echo -e "bw\ndelay\necho\nhz\ninfo\npub\ntype" | peco --prompt COMMAND\?\>)
        elif [ ${mode} = "service" ]; then
            local com=$(echo -e "args\ncall\nfind\ninfo\ntype\nuri" | peco --prompt COMMAND\?\>)
        elif [ ${mode} = "param" ]; then
            local com=$(echo -e "delete\ndump\nget\nload\nset" | peco --prompt COMMAND\?\>)
        fi
    elif [ ${mode} ] && [ $ROS_VERSION = 2 ]; then
        if [ ${mode} = "node" ]; then
            local var=$(ros2 node list | peco --prompt ROSNODE\?\>)
            local com=$(echo -e "info\nlog\nping\npub\nsub\nfind\nkill\ncleanup" | peco --prompt COMMAND\?\>)
        elif [ ${mode} = "topic" ]; then
            local var=$(ros2 topic list | peco --prompt ROSTOPIC\?\>)
            local com=$(echo -e "bw\ndelay\necho\nhz\ninfo\npub\ntype" | peco --prompt COMMAND\?\>)
        elif [ ${mode} = "service" ]; then
            local var=$(ros2 service list | peco --prompt ROSSERVICE\?\>)
            local com=$(echo -e "call\ntype\n" | peco --prompt COMMAND\?\>)
        fi
    fi
    if [ -n "${var}" ] && [ -n "${com}" ]; then
        if [ $ROS_VERSION = 1 ]; then
            local fullcmd="ros${mode} ${com} ${var}"
        elif [ $ROS_VERSION = 2 ]; then
            local fullcmd="ros2 ${mode} ${com} ${var}"
        fi
        RBUFFER="${fullcmd}"
        CURSOR=${#RBUFFER}
    fi
}

# ros2 completion
complete -o nospace -o default -F _python_argcomplete "ros2"

# colcon completion
type colcon &> /dev/null && source /usr/share/colcon_cd/function/colcon_cd.sh \
  && [[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]] \
  && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Hotfix for completion issue in jazzy. Known bug in https://github.com/ros2/ros2cli/issues/534
eval "$(register-python-argcomplete ros2)"
eval "$(register-python-argcomplete colcon)"

if [ -d /opt/ros ]; then
    init_ros_zsh
fi
