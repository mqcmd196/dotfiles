#!/bin/bash
GIT_SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)
GIT_HOME_CONFIGDIR=$HOME

set_gitignore_global(){
    file=gitignore_global
    ln -sf $GIT_SETUP_CONFIGDIR/$file $GIT_HOME_CONFIGDIR/.$file
    git config --global core.execludesFile $GIT_HOME_CONFIGDIR/.$file
}

main(){
    set_gitignore_global
}

main
