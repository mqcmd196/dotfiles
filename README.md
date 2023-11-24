# Debian/Ubuntu Settings for mqcmd196
![sudoer test](https://github.com/mqcmd196/dotfiles/actions/workflows/sudoer.yml/badge.svg?branch=master) ![non-sudoer test](https://github.com/mqcmd196/dotfiles/actions/workflows/non-sudoer.yml/badge.svg?branch=master)
## Supported OS
- Debian 11 (bullseye)
- Ubuntu 20.04 (focal fossa)

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

### Keybinds
#### emacs
C-; : completion

C-u C-SPC : back to the initial position when search

`lsp-workspace-restart` : Reload lsp at current workspace. It is convenient after the build 

##### project
C-c p f : Find file in project

##### vcs
C-c v g : Magit status

##### file
C-c f r : Find recently opened file
