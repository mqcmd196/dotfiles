#!/bin/bash
SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)

EMACS_CONFIGDIR="${SETUP_CONFIGDIR}/emacs"
VIM_CONFIGDIR="${SETUP_CONFIGDIR}/vim"

PERCOL_CONFIGDIR="${SETUP_CONFIGDIR}/percol"

BASH_CONFIGDIR="${SETUP_CONFIGDIR}/bash"
ZSH_CONFIGDIR="${SETUP_CONFIGDIR}/zsh"

TMUX_CONFIGDIR="${SETUP_CONFIGDIR}/tmux"

GIT_CONFIGDIR="${SETUP_CONFIGDIR}/git"

# editor settings
setup_editor(){
    echo "Setting up the editor configs..."
    $EMACS_CONFIGDIR/setup.sh # emacs
    ln -sf $VIM_CONFIGDIR/vimrc $HOME/.vimrc 	# vim
    echo "Finished to set the editor configs"
}

# percol settings
setup_percol(){
    echo "Setting up the percol configs..."
    $PERCOL_CONFIGDIR/setup.sh
    echo "Finished to set the percol configs"
}

# shell settings
setup_shell(){
    echo "Setting up the shell configs..."
    ln -sf $BASH_CONFIGDIR/bashrc $HOME/.bashrc
    ln -sf $ZSH_CONFIGDIR/zshrc $HOME/.zshrc
    echo "Finished to set the shell configs"
}

# tmux settings
setup_tmux(){
    echo "Setting up the tmux configs..."
    $TMUX_CONFIGDIR/setup.sh
    echo "Finished to set the tmux configs"
}

# git settings
setup_git(){
    echo "Setting up the git configs..."
    $GIT_CONFIGDIR/setup.sh
    echo "Finished to set the git configs"
}

main(){
    setup_editor
    setup_percol
    setup_shell
    setup_tmux
    setup_git
}

main
