#!/bin/sh

cp .bashrc ~/.bashrc_personal
echo '\nsource ~/.bashrc_personal' >> ~/.bashrc

cp .pythonrc ~/.pythonrc
cp .ripgreprc ~/.ripgreprc
cp .tmux.conf ~/.tmux.conf
cp .vimrc ~/.vimrc
