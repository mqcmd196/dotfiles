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
    if [ -e "/opt/ros/melodic/setup.zsh" ]; then
        source /opt/ros/melodic/setup.zsh # melodic only
    elif [ -e "/opt/ros/noetic/setup.zsh" ]; then
        source /opt/ros/noetic/setup.zsh
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
    ros_workspace_set $catkin_ws
}

rospeco() {
    local mode
    local var
    local com
    mode=$(echo -e "node\ntopic\nservice\nmsg" | peco --prompt ROS\?\>)
    if [ ${mode} ] && [ $ROS_VERSION = 1 ]; then
        if [ ${mode} = "node" ]; then
            local var=$(rosnode list | peco --prompt ROSNODE\?\>)
            local com=$(echo -e "ping\ninfo\nmachine\nkill\ncleanup" | peco --prompt COMMAND\?\>)
        elif [ ${mode} = "topic" ]; then
            local var=$(rostopic list | peco --prompt ROSTOPIC\?\>)
            local com=$(echo -e "bw\ndelay\necho\nhz\ninfo\npub\ntype" | peco --prompt COMMAND\?\>)
        elif [ ${mode} = "service" ]; then
            local var=$(rosservice list | peco --prompt ROSSERVICE\?\>)
            local com=$(echo -e "args\ncall\nfind\ninfo\ntype\nuri" | peco --prompt COMMAND\?\>)
        elif [ ${mode} = "msg" ]; then
            local var=$(rosmsg list | peco --prompt ROSMSG\?\>)
            local com=$(echo -e "show\ninfo\nmd5\npackage\npackages" | peco --prompt COMMAND\?\>)
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
if type colcon &> /dev/null; then
    source /usr/share/colcon_cd/function/colcon_cd.sh
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi

if [ -d /opt/ros ]; then
    init_ros_zsh
fi
