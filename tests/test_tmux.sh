#!/usr/bin/env sh
set -eu

socket=$(mktemp -d)/tmux.sock
trap 'tmux -S "$socket" kill-server 2>/dev/null || :; rm -f "$socket"; rmdir "${socket%/*}"' EXIT HUP INT TERM
tmux -S "$socket" -f "$HOME/.tmux.conf" new-session -d -s dotfiles
keys=$(tmux -S "$socket" list-keys -T copy-mode)
case $(uname -s) in
    Darwin) clipboard=pbcopy ;;
    *) clipboard='xsel -bi' ;;
esac
printf '%s\n' "$keys" | grep ' M-w .*copy-pipe-and-cancel' | grep "$clipboard"
printf '%s\n' "$keys" | grep ' C-c .*copy-pipe-and-cancel' | grep "$clipboard"
