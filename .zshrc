# Autosuggest
source /usr/local/share/zsh-autosuggestions/zsh-autosuggestions.zsh

# pyenv configuration
export PYENV_ROOT="$HOME/.pyenv"
export PATH="$PYENV_ROOT/bin:$PATH"
if command -v pyenv 1>/dev/null 2>&1; then
  eval "$(pyenv init -)"
fi

alias gtypist 'gtypist -w'

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

# Old C++ linking paths
#export PATH="/usr/local/opt/opencv@2/bin:$PATH"
#export LDFLAGS="-L/usr/local/opt/opencv@2/lib"
#export CPPFLAGS="-I/usr/local/opt/opencv@2/include"
#export PKG_CONFIG_PATH="/usr/local/opt/opencv@2/lib/pkgconfig"

#export PATH="/usr/local/opt/opencv@3/bin:$PATH"
#export LDFLAGS="-L/usr/local/opt/opencv@3/lib"
#export CPPFLAGS="-I/usr/local/opt/opencv@3/include"
#export PKG_CONFIG_PATH="/usr/local/opt/opencv@3/lib/pkgconfig"

# Add MATLAB to path
export PATH="$PATH:/Applications/MATLAB_R2020a.app/bin/"

#export DYLD_LIBRARY_PATH=/Applications/MATLAB_R2020a.app/extern/bin/maci64/:$DYLD_LIBRARY_PATH
#export DYLD_LIBRARY_PATH=/Applications/MATLAB_R2020a.app/bin/maci64/:$DYLD_LIBRARY_PATH
#export DYLD_LIBRARY_PATH=/usr/local/Cellar/libpng/1.6.37/lib/:$DYLD_LIBRARY_PATH
#export DYLD_LIBRARY_PATH=/usr/local/Cellar/libtiff/4.1.0/lib:$DYLD_LIBRARY_PATH
#DYLD_FALLBACK_LIBRARY_PATH

# Hacky fix to force some packages to look at the brew version instead of system version (brew link didn't work for some reason)
export DYLD_FALLBACK_LIBRARY_PATH=/usr/local/Cellar/freetype/2.10.1/lib/:$DYLD_LIBRARY_PATH
#export DYLD_LIBRARY_PATH=/System/Library/Frameworks/ImageIO.framework/Resources:$DYLD_LIBRARY_PATH
#export DYLD_FRAMEWORK_PATH=/System/Library/Frameworks/ImageIO.framework/Resources:$DYLD_FRAMEWORK_PATH

# Configure ripgrep
export RIPGREP_CONFIG_PATH=$HOME/.ripgreprc

# for Homebrew sbin
export PATH="/usr/local/sbin:$PATH"

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
export PROMPT="%(?.%F{green}√.%F{red}?%?)%f %n@%m %F{cyan}%1~%f %# "

# Configure fzf fuzzy searching
[ -f ~/.fzf.zsh ] && source ~/.fzf.zsh
export FZF_DEFAULT_OPTS="--height 40% --layout=reverse --border --preview '([[ -f {} ]] && (bat --style=numbers --color=always {} || cat {})) || ([[ -d {} ]] && (tree -C {} | less)) || echo {} 2> /dev/null | head -200'"

# For alt-c on mac (fzf open directory with cd)
bindkey "ç" fzf-cd-widget

# Use fd (https://github.com/sharkdp/fd) instead of the default find
# command for listing path candidates.
_fzf_compgen_path() {
  fd --hidden --follow --exclude ".git" . "$1"
}

# Use fd to generate the list for directory completion
_fzf_compgen_dir() {
  fd --type d --hidden --follow --exclude ".git" . "$1"
}

# allows command to follow symbolic links and not exclude hidden files
export FZF_DEFAULT_COMMAND='fd --type f --hidden --follow --exclude .git'


