source ~/.commonrc

# Autosuggest
# source /usr/local/share/zsh-autosuggestions/zsh-autosuggestions.zsh
source $HOMEBREW_PREFIX/share/zsh-autosuggestions/zsh-autosuggestions.zsh

# Use vi key bindings in ZSH
setopt vi
# Make Vi mode transitions faster (KEYTIMEOUT is in hundredths of a second)
export KEYTIMEOUT=1

zstyle ':completion:*' menu select
zmodload zsh/complist

# Use the vi navigation keys in menu completion
bindkey -M menuselect 'h' vi-backward-char
bindkey -M menuselect 'k' vi-up-line-or-history
bindkey -M menuselect 'l' vi-forward-char
bindkey -M menuselect 'j' vi-down-line-or-history


# Try to correct the spelling of commands
setopt correct

# Add MATLAB to path
export PATH="$PATH:/Applications/MATLAB_R2023a.app/bin/"

# Configure ripgrep
export RIPGREP_CONFIG_PATH=$HOME/.ripgreprc

export CLICOLOR=1

# Base16 Shell configuration
BASE16_SHELL="$HOME/.config/base16-shell/"
[ -n "$PS1" ] && \
    [ -s "$BASE16_SHELL/profile_helper.sh" ] && \
        eval "$("$BASE16_SHELL/profile_helper.sh")"

        autoload -Uz vcs_info
        precmd_vcs_info() { vcs_info }
        precmd_functions+=( precmd_vcs_info )
        setopt prompt_subst
        RPROMPT=\$vcs_info_msg_0_
        zstyle ':vcs_info:git:*' formats '%F{yellow}(%b) %r%f'
        zstyle ':vcs_info:*' enable git

# Styling for Command Prompt
# https://zsh.sourceforge.io/Doc/Release/Prompt-Expansion.html#Special-characters
export PROMPT="%(?.%F{green}√.%F{red}?%?)%f %n@%m %F{cyan}%~%f %# "

# # Configure fzf fuzzy searching
# [ -f ~/.fzf.zsh ] && source ~/.fzf.zsh
# export FZF_DEFAULT_OPTS="--height 40% --layout=reverse --border --preview '([[ -f {} ]] && (bat --style=numbers --color=always {} || cat {})) || ([[ -d {} ]] && (tree -C {} | less)) || echo {} 2> /dev/null | head -200'"

# # For alt-c on mac (fzf open directory with cd)
# bindkey "ç" fzf-cd-widget

# # Use fd (https://github.com/sharkdp/fd) instead of the default find
# # command for listing path candidates.
# _fzf_compgen_path() {
#   fd --hidden --follow --exclude ".git" . "$1"
# }

# # Use fd to generate the list for directory completion
# _fzf_compgen_dir() {
#   fd --type d --hidden --follow --exclude ".git" . "$1"
# }

# # allows command to follow symbolic links and not exclude hidden files
# export FZF_DEFAULT_COMMAND='fd --type f --hidden --follow --exclude .git'

