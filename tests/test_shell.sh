#!/usr/bin/env sh
set -eu

# Interactive startup must not hide missing commands or invalid options.
log=$(mktemp)
fresh_home=$(mktemp -d)
trap 'rm -f "$log"; rm -rf "$fresh_home"' EXIT HUP INT TERM
TERM=dumb zsh -lic 'exit' >"$log" 2>&1
# A fresh home must work without NVM or global startup files initializing compdef.
repo_dir=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)
ln -s "$repo_dir/roles/common/zsh/rc.zsh" "$fresh_home/.zshrc"
ln -s "$repo_dir/roles/common/zsh" "$fresh_home/.zsh.d"
# shellcheck disable=SC2016 # Expand variables inside zsh.
env HOME="$fresh_home" ZDOTDIR="$fresh_home" TERM=dumb zsh -dic '
    if [[ $OSTYPE == darwin* ]]; then
        (( ! $+functions[rospeco] )) || exit 1
    fi
    (( $+functions[compdef] )) || exit 1
' >>"$log" 2>&1
cat "$log"
if grep -Ei 'command not found|no such file|no such option|permission denied|parse error' "$log"; then
    exit 1
fi
TERM=dumb zsh -ic '
    ls >/dev/null || exit 1
    (( $+functions[_zsh_autosuggest_start] )) || exit 1
    (( $+functions[_zsh_highlight] )) || exit 1
    eml --batch --eval "(kill-emacs 0)"
'
