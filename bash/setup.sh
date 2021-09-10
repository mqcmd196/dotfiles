#!/bin/bash
BASH_SETUP_CONFIGDIR=$(cd $(dirname $0);pwd)
BASH_HOME_CONFIGDIR=$HOME/.bash.d

copy_bash_config(){
    echo "Copying the config files..."
    if [ ! -d $BASH_HOME_CONFIGDIR ]; then
        mkdir -p $BASH_HOME_CONFIGDIR
    fi
    ln -sf $BASH_SETUP_CONFIGDIR/rc.bash $HOME/.bashrc
    # Copy sourcing bash file
    for file in `\find ${BASH_SETUP_CONFIGDIR} -maxdepth 1 -type f`; do
        if [ $file != $BASH_SETUP_CONFIGDIR/setup.sh ] \
               &&  [ $file != $BASH_SETUP_CONFIGDIR/rc.bash ]; then
            ln -sf $file $BASH_HOME_CONFIGDIR
        fi
    done
    if [ ! -e $BASH_HOME_CONFIGDIR/local.bash ]; then
        touch $BASH_HOME_CONFIGDIR/local.bash
        echo "#!/bin/bash" > $BASH_HOME_CONFIGDIR/local.bash
        echo "# ### SET YOUR LOCAL BASHRC HERE ### " >> $BASH_HOME_CONFIGDIR/local.bash
    fi
}

main(){
    copy_bash_config
}

main
