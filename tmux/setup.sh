#!/bin/bash
TMUX_SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)
DOTFILES_ROOTDIR=$TMUX_SETUP_CONFIGDIR/../
TMUX_HOME_CONFIGDIR=$HOME

# install tmux
install_tmux(){
    sudo $DOTFILES_ROOTDIR/install_sysdeps -p tmux -y $DOTFILES_ROOTDIR/packages.yaml
}

copy_tmux_config(){
    echo "Copying the config files..."
    ln -sf $TMUX_SETUP_CONFIGDIR/tmux.conf $TMUX_HOME_CONFIGDIR/.tmux.conf
}

main(){
    install_tmux
    copy_tmux_config
}

main
