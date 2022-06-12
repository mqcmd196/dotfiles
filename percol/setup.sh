#!/bin/bash
PERCOL_SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)
DOTFILES_ROOTDIR=$EMACS_SETUP_CONFIGDIR/../
PERCOL_HOME_CONFIGDIR=$HOME/.percol.d

# install percol
install_percol(){
    echo "Installing the percol..."
    sudo -E $DOTFILES_ROOTDIR/install_sysdeps -r $PERCOL_SETUP_CONFIGDIR/required_pkgs.txt -y $DOTFILES_ROOTDIR/packages.yaml
}

copy_percol_config(){
    echo "Copying the config files..."
    if [ ! -d $PERCOL_HOME_CONFIGDIR ]; then
        mkdir -v -p $PERCOL_HOME_CONFIGDIR
    fi
    for file in `\find ${PERCOL_SETUP_CONFIGDIR} -maxdepth 1 -type f`; do
        if [ $file != $PERCOL_SETUP_CONFIGDIR/setup.sh ]; then
           ln -v -sf $file $PERCOL_HOME_CONFIGDIR
        fi
    done
}

main(){
    copy_percol_config
}

main
