#!/usr/bin/env sh
set -eu

# Interactive startup must not hide missing commands or invalid options.
log=$(mktemp)
trap 'rm -f "$log"' EXIT HUP INT TERM
TERM=dumb zsh -lic 'exit' >"$log" 2>&1
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
