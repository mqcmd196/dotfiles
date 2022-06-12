#!/bin/bash

SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)/non-sudoer
SETUP_ROOT_USER_CONFIGDIR=$(cd $(dirname $0);pwd)

ln -sf $SETUP_CONFIGDIR/rc.bash $HOME/.bashrc
ln -sf $SETUP_CONFIGDIR/emacs.el $HOME/.emacs
ln -sf $SETUP_ROOT_USER_CONFIGDIR/tmux/tmux.conf $HOME/.tmux.conf
ln -sf $SETUP_ROOT_USER_CONFIGDIR/git/gitignore_global $HOME/.gitignore_global
mkdir -p $HOME/.percol.d
ln -sf $SETUP_ROOT_USER_CONFIGDIR/percol/rc.py $HOME/.percol.d/rc.py

