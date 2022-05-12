#!/bin/bash

GDB_SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)
GDB_HOME_CONFIGDIR=$HOME

set_gdbinit(){
    file=.gdbinit
    ln -sf $GDB_SETUP_CONFIGDIR/gdb-dashboard/$file $GDB_HOME_CONFIGDIR/$file
}

main(){
    set_gdbinit
}
main
