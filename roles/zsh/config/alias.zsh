#!/usr/bin/env zsh

# editor
alias em='emacs'
alias eml='/usr/bin/emacs -q --load ~/.emacs-light.el'

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

# git
alias gfa='git fetch --all'
alias gco='git checkout'
alias ga='git add'
alias gcm='git commit -m'
alias gca='git commit --amend'
alias gp='git push'
alias gst='git status'
alias gd='git diff'

alias clangd='clangd-18'
alias clang-format='clang-format-18'

# explorer
if [ -v WSLENV ]; then
    alias e='explorer.exe'
else
    alias e='xdg-open'
fi

alias dquilt="quilt --quiltrc=${HOME}/.quiltrc-dpkg"
