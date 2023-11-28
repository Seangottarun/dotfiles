#!/bin/sh

cp .commonrc ~/.commonrc
cp .bashrc ~/.bashrc_personal
echo '\nsource ~/.bashrc_personal' >> ~/.bashrc

cp .pythonrc ~/.pythonrc
cp .ripgreprc ~/.ripgreprc
git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
cp .tmux.conf ~/.tmux.conf
echo "Remember to hit <prefix> I to install TPM plugins"
cp .vimrc ~/.vimrc
cp .ideavimrc ~/.ideavimrc

