#!/bin/bash


if [[ `id -u` -ne 0 ]]; then # Execute when root user

    echo "Executed as root user. It will install some packages to system"

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
        $BASH_CONFIGDIR/setup.sh
        ln -sf $ZSH_CONFIGDIR/rc.zsh $HOME/.zshrc
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

else

    echo "Executed as non root user. It will just make symbolic links your home directory."
    SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)/non-sudoer

    ln -v -sf $SETUP_CONFIGDIR/rc.bash $HOME/.bashrc
    ln -v -sf $SETUP_CONFIGDIR/emacs.el $HOME/.emacs
    ln -v -sf $SETUP_CONFIGDIR/tmux.conf $HOME/.tmux.conf
    ln -v -sf $SETUP_CONFIGDIR/gitignore_global $HOME/.gitignore_global
    mkdir -v -p $HOME/.percol.d
    ln -v -sf $SETUP_CONFIGDIR/percol_rc.py $HOME/.percol.d/rc.py

fi
