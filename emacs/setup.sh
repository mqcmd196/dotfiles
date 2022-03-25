#!/bin/bash
EMACS_SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)
DOTFILES_ROOTDIR=../$EMACS_SETUP_CONFIGDIR
EMACS_HOME_CONFIGDIR=$HOME/.emacs.d
DOOM_HOME_CONFIGDIR=$HOME/.doom.d

# install emacs
install_sysdeps(){
    echo "Installing sysdeps for Doom Emacs..."
    sudo $DOTFILES_ROOTDIR/install_sysdeps -r $EMACS_SETUP_CONFIGDIR/required_pkgs.txt -y $DOTFILES_ROOTDIR/packages.yaml
}

install_doom_emacs(){
    echo "Installing doom Emacs..."
    git clone --depth 1 https://github.com/hlissner/doom-emacs.git ~/.emacs.d/
    ~/.emacs.d/bin/doom doctor -y
    ~/.emacs.d/bin/doom install -y
    ~/.emacs.d/bin/doom sync -y
}

copy_doom_config(){
    echo "Copying the doom emacs config files"
    mkdir -v $DOOM_HOME_CONFIGDIR
    for file in `\find ${EMACS_SETUP_CONFIGDIR}/doom -maxdepth 1 -type f`; do
        ln -v -sf $file $DOOM_HOME_CONFIGDIR
    done
}

main(){
    install_sysdeps
    install_doom_emacs
    copy_doom_config
}

main
