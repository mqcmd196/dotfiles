#!/bin/bash

SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)/non-sudoer

ln -sf $SETUP_CONFIGDIR/rc.bash $HOME/.bashrc
ln -sf $SETUP_CONFIGDIR/emacs.el $HOME/.emacs
ln -sf $SETUP_CONFIGDIR/tmux.conf $HOME/.tmux.conf
ln -sf $SETUP_CONFIGDIR/gitignore_global $HOME/.gitignore_global
mkdir -p $HOME/.percol.d
ln -sf $SETUP_CONFIGDIR/percol_rc.py $HOME/.percol.d/rc.py

