#!/bin/bash
EMACS_SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)
EMACS_HOME_CONFIGDIR=$HOME/.emacs.d
DOOM_HOME_CONFIGDIR=$HOME/.doom.d
EMACS=emacs27

# install emacs
install_emacs(){
    echo "Installing the emacs27..."
    if [ "$(uname)" == 'Darwin' ]; then
        brew install emacs27
    elif [ "$(expr substr $(uname -s) 1 5)" == 'Linux' ]; then
        if [ $(lsb_release -r | awk '{print $2}') == '18.04' ]; then
            sudo add-apt-repository ppa:kelleyk/emacs && sudo apt update
            sudo apt install emacs27
	elif [ $(lsb_release -r | awk '{print $2}') == '20.04' ]; then
	    sudo add-apt-repository ppa:kelleyk/emacs && sudo apt update
            sudo apt install emacs27
        fi
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
    ln -sf $EMACS_SETUP_CONFIGDIR/init.el $EMACS_HOME_CONFIGDIR
    ln -sf $EMACS_SETUP_CONFIGDIR/conf $EMACS_HOME_CONFIGDIR
}

install_doom_emacs(){
    echo "Installing sysdeps for Doom Emacs..."
    sudo apt install cmigemo ripgrep fd-find silversearcher-ag
    echo "Installing Doom Emacs..."
    git clone --depth 1 https://github.com/hlissner/doom-emacs.git ~/.emacs.d/
    ~/.emacs.d/bin/doom doctor
    ~/.emacs.d/bin/doom install
    ~/.emacs.d/bin/doom sync
}

copy_doom_config(){
    echo "Copying the doom emacs config files"
    for file in `\find ${EMACS_SETUP_CONFIGDIR}/doom -maxdepth 1 -type f`; do
        ln -sf $file $DOOM_HOME_CONFIGDIR
    done
}

install_ccls(){
    echo "Installing the ccls..."
    ppwd=$(pwd)
    echo "Go to ccls path..."
    cd $EMACS_SETUP_CONFIGDIR
    git submodule update --init --recursive
    if [ "$(expr substr $(uname -s) 1 5)" == 'Linux' ]; then
        if [ $(lsb_release -r | awk '{print $2}') == '18.04' ]; then
            echo "Building the ccls..."
            wget -c http://releases.llvm.org/8.0.0/clang+llvm-8.0.0-x86_64-linux-gnu-ubuntu-18.04.tar.xz
            tar xf clang+llvm-8.0.0-x86_64-linux-gnu-ubuntu-18.04.tar.xz
            cd ccls
            cmake -H. -BRelease -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=../clang+llvm-8.0.0-x86_64-linux-gnu-ubuntu-18.04 -DUSE_SYSTEM_RAPIDJSON=OFF
            cmake --build Release
            cd -
	      elif [ $(lsb_release -r | awk '{print $2}') == '20.04' ]; then
	          sudo apt-get install clang libclang-10-dev ccls
        fi
    fi
    cd $ppwd
}

main(){
    if !(type emacs27 > /dev/null 2>&1); then
        install_emacs
    fi
    install_ccls
    install_doom_emacs
    copy_doom_config
}

main
