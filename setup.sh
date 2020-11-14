#!/bin/sh
ln -sf ~/dotfiles/.vimrc ~/.vimrc
ln -sf ~/dotfiles/.emacs ~/.emacs
ln -sf ~/dotfiles/.bashrc ~/.bashrc
ln -sf ~/dotfiles/.gitignore_global ~/.gitignore_global
git config --global core.execludesFile ~/.gitignore_global
