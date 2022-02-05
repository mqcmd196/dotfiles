#!/bin/bash
PERCOL_SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)
PERCOL_HOME_CONFIGDIR=$HOME/.percol.d

# install percol
install_percol(){
    echo "Installing the percol..."
    if [ "$(uname)" == 'Darwin' ]; then
        pip2 install percol
    elif [ "$(expr substr $(uname -s) 1 5)" == 'Linux' ]; then
        sudo pip3 install percol
    else
        echo "Not supported OS"
        exit 1
    fi
}

copy_percol_config(){
    echo "Copying the config files..."
    if [ ! -d $PERCOL_HOME_CONFIGDIR ]; then
        mkdir -p $PERCOL_HOME_CONFIGDIR
    fi
    for file in `\find ${PERCOL_SETUP_CONFIGDIR} -maxdepth 1 -type f`; do
        if [ $file != $PERCOL_SETUP_CONFIGDIR/setup.sh ]; then
           ln -sf $file $PERCOL_HOME_CONFIGDIR
        fi
    done
}

main(){
    if !(type percol > /dev/null 2>&1); then
        install_percol
    fi
    copy_percol_config
}

main
