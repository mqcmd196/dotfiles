#!/bin/bash
TMUX_SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)
TMUX_HOME_CONFIGDIR=$HOME

# install tmux
install_tmux(){
    echo "Installing the tmux..."
    if [ "$(uname)" == 'Darwin' ]; then
        brew install tmux
    elif [ "$(expr substr $(uname -s) 1 5)" == 'Linux' ]; then
        sudo apt install tmux
    else
        echo "Not supported OS"
        exit 1
    fi
}

copy_tmux_config(){
    echo "Copying the config files..."
    ln -sf $TMUX_SETUP_CONFIGDIR/tmux.conf $TMUX_HOME_CONFIGDIR/.tmux.conf
}

main(){
    if !(type tmux > /dev/null 2>&1); then
        install_tmux
    fi
    copy_tmux_config
}

main
