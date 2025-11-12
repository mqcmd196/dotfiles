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
    if [[ $ROS_VERSION = 1 ]]; then
        local tmpd=$(mktemp -d)
        (
            setopt nomonitor nonotify
            rosnode    list >"$tmpd/nodes"    2>/dev/null & p1=$!
            rostopic   list >"$tmpd/topics"   2>/dev/null & p2=$!
            rosservice list >"$tmpd/services" 2>/dev/null & p3=$!
            rosparam   list >"$tmpd/params"   2>/dev/null & p4=$!
            wait $p1 $p2 $p3 $p4
        )
        local -a nodes topics services params
        nodes=("${(@f)$(<"$tmpd/nodes")}")
        topics=("${(@f)$(<"$tmpd/topics")}")
        services=("${(@f)$(<"$tmpd/services")}")
        params=("${(@f)$(<"$tmpd/params")}")
        rm -rf -- "$tmpd"
        local -a elements
        for i in $nodes;    do elements+="$i (node)";    done
        for i in $topics;   do elements+="$i (topic)";   done
        for i in $services; do elements+="$i (service)"; done
        for i in $params;   do elements+="$i (param)";   done
        local select mode var com
        select=$(printf '%s\n' "${elements[@]}" | peco --prompt 'ROS?>')
        mode=${${select##* }#\(}; mode=${mode%\)}
        var=${select%% *}
        case $mode in
            node)    com=$(printf '%s\n' info ping kill machine cleanup | peco --prompt 'COMMAND?>');;
            topic)   com=$(printf '%s\n' info echo hz pub bw delay type | peco --prompt 'COMMAND?>');;
            service) com=$(printf '%s\n' info call args find type uri   | peco --prompt 'COMMAND?>');;
            param)   com=$(printf '%s\n' get set dump delete load       | peco --prompt 'COMMAND?>');;
        esac
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
