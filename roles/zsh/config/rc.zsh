if [ -v WSLENV ]; then
  source $HOME/.profile # In wsl-22.04 sometimes .profile is not loaded
  export LIBGL_ALWAYS_SOFTWARE=1 # For showing robot model in rviz in WSL2
fi

# Enable Powerlevel10k instant prompt. Should stay close to the top of ~/.zshrc.
# Initialization code that may require console input (password prompts, [y/n]
# confirmations, etc.) must go above this block; everything else may go below.
if [[ -r "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh" ]]; then
  source "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh"
fi

source ~/.powerlevel10k/powerlevel10k.zsh-theme

### POWERLINE10K CONFIGS
# To customize prompt, run `p10k configure` or edit ~/.p10k.zsh.
[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh

# User configuration
# Preferred editor for local and remote sessions
if [[ -n $SSH_CONNECTION ]]; then
  export EDITOR='emacs -nw'
else
  export EDITOR='emacs -nw'
fi

### NVM CONFIGS
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion

### KEYBINDINGS
# for using peco on reverse-i-search
alias peco="TERM=screen peco"
function peco-history-selection() {
    local tac
    if which tac > /dev/null; then
        tac="tac"
    else
        tac="tail -r"
    fi

    BUFFER=$(\history -n 1 | \
        eval $tac | \
        peco --query "$LBUFFER")
    CURSOR=$#BUFFER
    zle clear-screen
}
zle -N peco-history-selection
bindkey '^r' peco-history-selection

bindkey -e # emacs like keybinding
WORDCHARS='*?[]~&;!#$%^(){}<>' # for word jumping

autoload -U +X bashcompinit && bashcompinit # enable complete command
local zsh_personal_config_dir="$HOME/.zsh.d"
source ${zsh_personal_config_dir}/alias.zsh
source ${zsh_personal_config_dir}/ros.zsh
source ${zsh_personal_config_dir}/grc.zsh
if [ -f ${zsh_personal_config_dir}/local.zsh ]; then
  source ${zsh_personal_config_dir}/local.zsh
fi

# history
if [ -z "$HISTFILE" ]; then
  HISTFILE=$HOME/.zsh_history
fi

setopt histappend
setopt inc_append_history
setopt hist_verify
setopt extended_history
setopt hist_ignore_dups
setopt hist_ignore_space
setopt hist_ignore_all_dups

PROMPT_COMMAND='history -a'
HISTSIZE=500000
SAVEHIST=100000
HISTORY_IGNORE="(exit|l[sal]|bg|fg|history|clear|pwd|l)"
zshaddhistory(){
  emulate -L zsh
   [[ $1 != ${~HISTORY_IGNORE} ]]
}
HIST_STAMPS='yyyy-mm-dd HH:MM:SS'

source /etc/zsh_command_not_found # for command-not-found

# enable comment out in command line
setopt interactivecomments

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('$HOME/miniconda3/bin/conda' 'shell.zsh' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
        . "$HOME/miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="$HOME/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<
# conda-zsh-completion
fpath+=~/.conda-zsh-completion
zstyle ":conda_zsh_completion:*" sort-envs-by-time true

# try zsh completion
source ~/.local/src/try/completions/try.bash

# init completions
autoload -U compinit && compinit

# zsh syntax highlighting
source /usr/share/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh
ZSH_HIGHLIGHT_STYLES[comment]=fg=245 # make comment out color lighter

# zsh autosuggestions
source /usr/share/zsh-autosuggestions/zsh-autosuggestions.zsh
ZSH_AUTOSUGGEST_STRATEGY=(completion history)
ZSH_AUTOSUGGEST_USE_ASYNC=1 # avoid too slow completion
