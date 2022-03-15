################################################################################
# Global settings
################################################################################

################################################################################
# Use vim where possible
################################################################################
# vi-mode in bash
# Supports ^,$,b,e,w,d,y,p
# Enter with <Esc>, v to edit cmd in vim (:wq to run edited cmd)
set -o vi

export EDITOR=vim
# export EDITOR=nvim

# Use vim as pager
export MANPAGER="env MAN_PN=1 vim -M +MANPAGER -"
# export MANPAGER="nvim -c 'set ft=man' -"
# export PAGER="nvim -R"

# Working for nvim
# export MANPAGER='nvim +Man!'
# for cppman, use cppman -c system
# export PAGER='nvim +Man!'

function dirdiff() {
    vim -c "DirDiff $1 $2"
}

# append to the history file, don't overwrite it
shopt -s histappend

# If set, the pattern '**' used in a filename expansion context will match
# all files and zero or more directories and subdirectories. If the pattern
# is followed by a '/', only directories and subdirectories match.
# shopt -s globstar

# If set, an argument to the cd builtin command that is not a directory is
# assumed to be the name of a variable whose value is the directory to change
# to.
shopt -s cdable_vars

# If set, a command name that is the name of a directory is executed as
# if it were the argument to the cd command. This option is only used by
# interactive shells.
shopt -s autocd

# If set, Bash attempts spelling correction on directory names during word
# completion if the directory name initially supplied does not exist.
shopt -s dirspell

################################################################################
# History management
################################################################################

# verify commands after using !<index> or !<partial_cmd>
# Use history | less to get index
# !<partial_cmd> matches 1 cmd containing <partial_cmd>
# TODO: how to iterate through <partial_cmd> matches?
# maybe just use <c-r>?
shopt -s histverify

# append to the history file, don't overwrite it
shopt -s histappend

# history file
# export HISTFILE=
export HISTTIMEFORMAT="%Y/%m/%d %H:%M:%S:   "
export HISTSIZE=50000
export HISTFILESIZE=50000

################################################################################
# Miscellaneous
################################################################################
# Make bash completion work with :: for cppman
# This breaks tab completion for paths
# export COMP_WORDBREAKS=" /\"\'><;|&("

################################################################################
# Aliases
################################################################################
alias ..="cd .."
alias ...="cd ../.."
alias ....="cd ../../.."
alias .....="cd ../../../.."
alias ......="cd ../../../../.."
alias .......="cd ../../../../../.."

alias ls="ls --color -a"

# mkdir + cd
function mcd() {
    mkdir -p $*
    cd $*
    pwd
}
export -f mcd
