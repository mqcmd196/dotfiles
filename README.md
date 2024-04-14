# Debian/Ubuntu Settings for mqcmd196
![sudoer test](https://github.com/mqcmd196/dotfiles/actions/workflows/sudoer.yml/badge.svg?branch=master) ![non-sudoer test](https://github.com/mqcmd196/dotfiles/actions/workflows/non-sudoer.yml/badge.svg?branch=master)
## Supported OS
- Debian 11 (bullseye)
- Ubuntu 20.04 (focal fossa)
- Ubuntu 22.04 (Jammy Jellyfish)

## Prerequisite
### apt packages
```bash
sudo apt install ansible git
```

### clone
```bash
cd ~
git clone --recursive https://github.com/mqcmd196/dotfiles.git
```

## Setup
### When you are authorized to install packages with sudo
```bash
ansible-playbook setup_sudoer.yml -K 
```

#### If you want to execute specific task
```bash
ansible-playbook setup_sudoer.yml -K --tags emacs # e.g. emacs 
```

#### If you want to check full logs
```bash
ansible-playbook setup_sudoer.yml -K -vvv 
```

### When you are not authorized to install packages with sudo
Limited dotfiles
``` bash
./setup_nonsudoer
```

## Note
### Initial setup in sudoer emacs
1. Execute `copilot-login` when `copilot-mode` is enabled
1. On emacs, execute `all-the-icons-install-fonts` and `nerd-icons-install-fonts`

### Manuals of functions and keybinds
#### emacs
C-; : Completion

C-u \<n\> : Repeat the next command `n` times. (n=4 by default)

C-u C-SPC : Back to the initial position when search

M-[ : Return to the beginning of the paragraph that precedes the point

M-] : Move to the end of the paragraph behind the point

M-h : Place a point and mark before or after the paragraph with the point or the paragraph following the point

`lsp-workspace-restart` : Reload lsp at current workspace. It is convenient after the build 

`reftex-toc` : Show the table of contents for the current tex document 

##### project
C-c p f : Find file in project

##### vcs
C-c v g : Magit status

##### file
C-c f r : Find recently opened file

## LICENSE
This repository includes emacs-one-themes which is licensed under GPL-3.0. If you don't want to use it, you can use it under the BSD-3-Clause license. 
