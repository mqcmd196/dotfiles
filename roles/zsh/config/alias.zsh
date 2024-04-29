#!/usr/bin/env zsh

# editor
alias em='emacs'
alias eml='/usr/bin/emacs -q --load ~/.emacs-light.el'
alias emll="OS_COLOR_SCHEME='light' /usr/bin/emacs -q --load ~/.emacs-light.el"
alias emld="OS_COLOR_SCHEME='dark' /usr/bin/emacs -q --load ~/.emacs-light.el"

alias doom='~/.emacs.d/bin/doom'

# tmux
alias t='tmux'

# build
alias m='make'
alias n='ninja'

if type "batcat" > /dev/null 2>&1; then
    alias bat="batcat"
fi

# ls
alias ls='ls --color=auto'
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# grep
alias grep='grep --color=auto'
alias fgrep='fgrep --color=auto'
alias egrep='egrep --color=auto'

# colorize some commands output
if type "grc" > /dev/null 2>&1; then
    alias ip='grc ip'
    alias ifconfig='grc ifconfig'
    alias ping='grc ping'
    alias traceroute='grc traceroute'
    alias netstat='grc netstat'
    alias ps='grc ps'
    alias df='grc df'
fi

# git
alias gfa='git fetch --all'
alias gco='git checkout'
alias ga='git add'
alias gcm='git commit -m'
alias gca='git commit --amend'
alias gp='git push'
alias gst='git status'
alias gd='git diff'

source /etc/lsb-release
alias clangd='clangd-18'
alias clang-format='clang-format-18'

# explorer
if [ -v WSLENV ]; then
    alias e='explorer.exe'
else
    alias e='xdg-open'
fi
