# ################################### OH-MY-BASH CONFIGS #################################
# See oh-my-bash/templates/bashrc.osh-template
export OSH=$HOME/.bash.d/oh-my-bash
OSH_THEME="agnoster"
completions=(
    git
    composer
    ssh
)
aliases=(
  general
)
plugins=(
  git
  bashmarks
)
source $OSH/oh-my-bash.sh
# ################################# END OH-MY-BASH CONFIGS ###############################


# ################################### PERSONAL CONFIGS ###################################
BASH_PERSONAL_CONFIGDIR=$HOME/.bash.d

# for using percol on reverse-i-search
_replace_by_history() {
 local l=$(HISTTIMEFORMAT= history | tac | sed -e 's/^\s*[0-9]\+\s\+//' | percol --query "$READLINE_LINE")
 READLINE_LINE="$l"
 READLINE_POINT=${#l}
}
bind -x '"\C-r": _replace_by_history'

# share history on tmux
# function share_history {
#     history -a
#     history -c
#     history -r
# }
# PROMPT_COMMAND='share_history'
# shopt -u histappend

# use emacs27 as default
if (type emacs27 > /dev/null 2>&1); then
    alias emacs='emacs27'
fi

alias e='emacs'

source ${BASH_PERSONAL_CONFIGDIR}/ros.bash
source ${BASH_PERSONAL_CONFIGDIR}/local.bash

unset BASH_PERSONAL_CONFIGDIR
# ################################# END PERSONAL CONFIGS #################################
