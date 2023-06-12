#!/usr/bin/env zsh

alias em='emacs'
alias doom='~/.emacs.d/bin/doom'

if type "batcat" > /dev/null 2>&1; then
    alias bat="batcat"
fi

alias ls='ls --color=auto'
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

alias grep='grep --color=auto'
alias fgrep='fgrep --color=auto'
alias egrep='egrep --color=auto'

alias ip='grc ip'
alias ifconfig='grc ifconfig'
alias ping='grc ping'
alias traceroute='grc traceroute'
alias netstat='grc netstat'
alias ps='grc ps'
alias df='grc df'
