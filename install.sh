#!/bin/sh

cp .commonrc ~/.commonrc
cp .bashrc ~/.bashrc_personal
# Manually append a blank line
# NB: Using \n with echo adds a literal \n instead of line break
echo '' >> ~/.bashrc
echo 'source ~/.bashrc_personal' >> ~/.bashrc

cp .pythonrc ~/.pythonrc
cp .ripgreprc ~/.ripgreprc
git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
cp .tmux.conf ~/.tmux.conf
echo "Remember to hit <prefix> I to install TPM plugins"
cp .vimrc ~/.vimrc
cp .ideavimrc ~/.ideavimrc

