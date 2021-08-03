# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

if [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
fi

# for showing git branch at the current directory
if [ -f /etc/bash_completion.d/git-prompt ]; then
    export PS1='\[\033[01;32m\]\u@\h\[\033[01;33m\] \w$(__git_ps1) \n\[\033[01;34m\]\$\[\033[00m\] '
else
    export PS1='\[\033[01;32m\]\u@\h\[\033[01;33m\] \w \n\[\033[01;34m\]\$\[\033[00m\] '
fi

# for using percol on reverse-i-search
_replace_by_history() {
 local l=$(HISTTIMEFORMAT= history | tac | sed -e 's/^\s*[0-9]\+\s\+//' | percol --query "$READLINE_LINE")
 READLINE_LINE="$l"
 READLINE_POINT=${#l}
}
bind -x '"\C-r": _replace_by_history'

# share history on tmux
function share_history {
    history -a
    history -c
    history -r
}
PROMPT_COMMAND='share_history'
shopt -u histappend

# source `catkin locate --shell-verbs`
echo "CMAKE_PREFIX_PATH: ""$CMAKE_PREFIX_PATH"

# for using ROS2 on macOS
if [ "$(uname)" == 'Darwin' ]; then
    export OPENSSL_ROOT_DIR=/usr/local/opt/openssl@1.1
fi

# ################################### START WORKSPACES ###################################
export WORKSPACES_ROOT=${HOME}
# export WORKSPACES_ROOT="/home/mujin"

mujin_workspace_make () {
    CURRENT_WORKING_DIR=$(pwd)
    MUJIN_WORKSPACE_ID=$1

    mkdir -p ${WORKSPACES_ROOT}/workspaces/${MUJIN_WORKSPACE_ID}/checkoutroot
    cd ${WORKSPACES_ROOT}/workspaces/${MUJIN_WORKSPACE_ID}
    git clone git@git.mujin.co.jp:jhbuild/jhbuildcommon.git
    git clone git@git.mujin.co.jp:jhbuild/jhbuildappcontroller.git
    mujin_workspace_set ${MUJIN_WORKSPACE_ID}
    mujin_jhbuildcommon_updatejhbuildcommon.bash
    cd ${MUJINJH_APPCONTROLLER_HOME}
    mujin_jhbuildcommon_initjhbuild.bash
    jhbuild sysdeps --install
    mujin_workspace_set ${MUJIN_WORKSPACE_ID}
    cd ${CURRENT_WORKING_DIR}
}

mujin_workspace_set() {
    MUJIN_WORKSPACE_ID=$1
    MUJIN_WORKSPACE_ROOT=${WORKSPACES_ROOT}/workspaces/${MUJIN_WORKSPACE_ID}

    # Clear previous MUJIN env variables
    unset CMAKE_PREFIX_PATH
    unset PYTHONPATH
    unset QML_IMPORT_PATH
    unset OPENRAVE_PLUGINS
    unset HDF5_PLUGIN_PATH
    unset GENICAM_GENTL64_PATH
    unset QML2_IMPORT_PATH
    unset OPENRAVE_DATA
    unset PKG_CONFIG_PATH

    unset MUJIN_SHUGO_SHARE_DIR

    unset MUJIN_TESTAPPCONTROLLER_SHARE_DIR
    unset MUJIN_RESOURCES_SHARE_DIR
    unset MUJIN_ROBOTS_SHARE_DIR
    unset QT_PLUGIN_PATH
    unset MUJIN_APPCONTROLLER_HOME
    unset MUJIN_APPFRPIECEPICKINGWITHQPS_SHARE_DIR
    unset MUJIN_TESTREGISTRATION_SHARE_DIR

    MUJIN_TESTAPPCONTROLLER_SHARE_DIR=${MUJIN_WORKSPACE_ROOT}/checkoutroot/appcontroller/testappcontroller
    MUJIN_RESOURCES_SHARE_DIR=${MUJIN_WORKSPACE_ROOT}/checkoutroot/resources
    MUJIN_ROBOTS_SHARE_DIR=${MUJIN_WORKSPACE_ROOT}/checkoutroot/robots
    QT_PLUGIN_PATH=${MUJIN_WORKSPACE_ROOT}/${MUJIN_WORKSPACE_JHBUILDAPP_NAME}/install/lib/plugins:
    MUJIN_APPCONTROLLER_HOME=${MUJIN_WORKSPACE_ROOT}/checkoutroot/appcontroller
    MUJIN_APPFRPIECEPICKINGWITHQPS_SHARE_DIR=${MUJIN_WORKSPACE_ROOT}/checkoutroot/appfrpiecepickingwithqps
    MUJIN_TESTREGISTRATION_SHARE_DIR=${MUJIN_WORKSPACE_ROOT}/checkoutroot/registration/testregistration

    for VAR in `printenv | grep ^MUJIN | sed -e 's/=.*$//'`; do
        unset -f ${VAR}
    done
    export MUJIN_WORKSPACE_JHBUILDAPP_NAME="jhbuildappcontroller"
    echo -e "Configuring Mujin environment: \033[01;32m${MUJIN_WORKSPACE_ID}\033[00m"
    # Update .jhbuildrc
    if [ "$(diff -q ${MUJIN_WORKSPACE_ROOT}/${MUJIN_WORKSPACE_JHBUILDAPP_NAME}/.jhbuildrc ${HOME}/.jhbuildrc)" != "" ]; then
        echo "Updating ~/.jhbuildrc"
        cp ${MUJIN_WORKSPACE_ROOT}/${MUJIN_WORKSPACE_JHBUILDAPP_NAME}/.jhbuildrc ${HOME}/
    fi
    export LD_LIBRARY_PATH="/usr/local/lib"
    unset PATH
    export PATH=$ORG_PATH:/usr/local/bin:/usr/bin:/bin:$HOME/.local/bin
    export PATH=/usr/lib/ccache:$PATH
    YELLOW='\033[1;33m'
    GREEN='\033[0;32m'
    RED='\033[0;31m'
    NC='\033[0m' # No Color
    display_text() {
        # $1 color
        # $2 text
        echo -e "$1$2${NC}"
    }
    log_info() {
        display_text $NC "$1"
    }
    log_debug() {
        display_text $GREEN "$1"
    }
    log_warn() {
        display_text $YELLOW "$1"
    }
    log_error() {
        display_text $RED "$1"
    }
    if [[ $PATH != *"${MUJIN_WORKSPACE_ROOT}/jhbuildcommon/bin"* ]]; then
        export PATH="${MUJIN_WORKSPACE_ROOT}/jhbuildcommon/bin:$PATH"
    fi
    export MUJINJH_COMMON=${MUJIN_WORKSPACE_ROOT}/jhbuildcommon
    if [ -f "${MUJINJH_COMMON}/jhbuild/contrib/jhbuild_completion.bash" ]; then
        source "${MUJINJH_COMMON}/jhbuild/contrib/jhbuild_completion.bash"
    fi

    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]$(__git_ps1) \[\033[01;32m\](workspace:${MUJIN_WORKSPACE_ID}) \[\033[00m\]$ '
    export MUJIN_JHBUILD_CHECKOUT_DIR=${MUJIN_WORKSPACE_ROOT}/checkoutroot
    export MUJINJH_APPCONTROLLER_HOME=${MUJIN_WORKSPACE_ROOT}/${MUJIN_WORKSPACE_JHBUILDAPP_NAME}
    export D=${MUJIN_WORKSPACE_ROOT}/checkoutroot/detectors/python/mujindetection/
    # alias unitest="MUJIN_RESOURCES_DIR=/testdata mujin_testcontrollercommon_runpytest.py ${MUJIN_WORKSPACE_ROOT}/checkoutroot/detectors/testdetectors/python/mujintestdetection -m askulbox0*fast"
    # alias pytst="pytest -p no:cacheprovider -p no:mujintestcommon -v --log-level ERROR -m detector_ci_marker ${MUJIN_WORKSPACE_ROOT}/checkoutroot/testdetectors/python/mujintestdetection/test_detector.py --resource "
    # alias jb="cd ~/mujin/jhbuildappcontroller/; git checkout master; git pull origin master; mujin_jhbuildcommon_updatejhbuildcommon.bash; mujin_jhbuildcommon_initjhbuild.bash; jhbuild sysdeps --install; jhbuild"
    export MUJIN_JHBUILD_DOWNLOADS_DIR=${MUJIN_WORKSPACE_ROOT}/downloads
    source ${MUJINJH_APPCONTROLLER_HOME}/setuptestdev.bash

    return 0
}

mujin_workspace_checkout() {
    CURRENT_WORKING_DIR=$(pwd)
    MUJIN_WORKSPACE_ID=$1
    MUJIN_WORKSPACE_ROOT=${WORKSPACES_ROOT}/workspaces/${MUJIN_WORKSPACE_ID}
    rm ${HOME}/mujin || true
    ln -s ${MUJIN_WORKSPACE_ROOT} ${HOME}/mujin
    cd ${CURRENT_WORKING_DIR}
}

if [ -f ${WORKSPACES_ROOT} ]; then
    alias mujin_workspace_list='ls --color=never ${WORKSPACES_ROOT}/workspaces'
    CURRENT_WORKSPACES=`mujin_workspace_list`
    complete -W "${CURRENT_WORKSPACES}" mujin_workspace_set
    complete -W "${CURRENT_WORKSPACES}" mujin_workspace_checkout
fi
# ################################### END WORKSPACES ###################################

# Check out mujin workspace by default
mujin_workspace_set mujin
mujin_workspace_checkout mujin


# ################################### START PERSONAL ###################################
# alias vdd="mujin_testdetectors_rundetector.py --show --datadetectiondir ."
# alias vddl="mujin_testdetectors_rundetector.py --forcelocaldatadetection True --datadetectiondir . --reportname hamdiSingleDetTest"
# alias kk="kill -9 %"
# alias vnc="vinagre"
# alias title='f(){ echo -ne "\033]0;$@\007"; unset -f f; }; f'
# alias gitdiff='f(){ git diff `git merge-base origin/master $@` $@; unset -f f; }; f'
# alias git='f(){ sg net "/usr/bin/git $@"; unset -f f; }; f'
# ################################### END PERSONAL ###################################

export PATH=$PATH:~/.local/bin
source ~/mujin/jhbuildcommon/setup.bash
