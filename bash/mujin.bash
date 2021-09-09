#!/bin/bash

# FOR ENABLING THE WORKSPACE CONFIG AT MUJIN
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

# ################################### START PERSONAL ###################################
# alias vdd="mujin_testdetectors_rundetector.py --show --datadetectiondir ."
# alias vddl="mujin_testdetectors_rundetector.py --forcelocaldatadetection True --datadetectiondir . --reportname hamdiSingleDetTest"
# alias kk="kill -9 %"
# alias vnc="vinagre"
# alias title='f(){ echo -ne "\033]0;$@\007"; unset -f f; }; f'
# alias gitdiff='f(){ git diff `git merge-base origin/master $@` $@; unset -f f; }; f'
# alias git='f(){ sg net "/usr/bin/git $@"; unset -f f; }; f'
# ################################### END PERSONAL ###################################

# EXAMPLES:Check out mujin workspace by default
# mujin_workspace_set mujin
# mujin_workspace_checkout mujin
