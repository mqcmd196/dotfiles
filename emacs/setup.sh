#!/bin/bash
EMACS_SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)
EMACS_HOME_CONFIGDIR=$HOME/.emacs.d

# install emacs
install_emacs(){
    echo "Installing the emacs..."
    if [ "$(uname)" == 'Darwin' ]; then
        brew install emacs
    elif [ "$(expr substr $(uname -s) 1 5)" == 'Linux' ]; then
        sudo apt install emacs
    else
        echo "Not supported OS"
        exit 1
    fi   
}

copy_emacs_config(){
    echo "Copying the config files..."
    if [ ! -d $EMACS_HOME_CONFIGDIR ]; then
        mkdir -p $EMACS_HOME_CONFIGDIR
    fi
    for file in `\find ${EMACS_SETUP_CONFIGDIR} -maxdepth 1 -type f`; do
        if [ $file != $EMACS_SETUP_CONFIGDIR/setup.sh ]; then
           ln -sf $file $EMACS_HOME_CONFIGDIR
        fi
    done
}

main(){
    if !(type emacs > /dev/null 2>&1); then
        install_emacs
    fi
    copy_emacs_config
}

main
