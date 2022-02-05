#!/bin/bash
EMACS_SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)
EMACS_HOME_CONFIGDIR=$HOME/.emacs.d
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

install_ccls(){
    echo "Installing the ccls..."
    ppwd=$(pwd)
    echo "Go to ccls path..."
    cd $EMACS_SETUP_CONFIGDIR
    git submodule update --init --recursive
    echo "Building the ccls..."
    if [ "$(expr substr $(uname -s) 1 5)" == 'Linux' ]; then
        if [ $(lsb_release -r | awk '{print $2}') == '18.04' ]; then
            wget -c http://releases.llvm.org/8.0.0/clang+llvm-8.0.0-x86_64-linux-gnu-ubuntu-18.04.tar.xz
            tar xf clang+llvm-8.0.0-x86_64-linux-gnu-ubuntu-18.04.tar.xz
            cd ccls
            cmake -H. -BRelease -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=../clang+llvm-8.0.0-x86_64-linux-gnu-ubuntu-18.04 -DUSE_SYSTEM_RAPIDJSON=OFF
            cmake --build Release
            cd -
	elif [ $(lsb_release -r | awk '{print $2}') == '20.04' ]; then
	    sudo apt-get install clang libclang-10-dev
	    cd ccls
	    cmake -H. -BRelease -DCMAKE_BUILD_TYPE=Release \
		  -DCMAKE_PREFIX_PATH=/usr/lib/llvm-10 \
		  -DLLVM_INCLUDE_DIR=/usr/lib/llvm-10/include \
		  -DLLVM_BUILD_INCLUDE_DIR=/usr/include/llvm-10/
        fi
    fi
    cd $ppwd
}

main(){
    if !(type emacs27 > /dev/null 2>&1); then
        install_emacs
    fi
    install_yaml_mode
    install_ccls
    copy_emacs_config
}

main
