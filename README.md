# Debian/Ubuntu Settings for mqcmd196
![sudoer test](https://github.com/mqcmd196/dotfiles/actions/workflows/sudoer.yml/badge.svg?branch=master) ![non-sudoer test](https://github.com/mqcmd196/dotfiles/actions/workflows/non-sudoer.yml/badge.svg?branch=master)
## Supported OS
- Debian 11 (bullseye)
- Ubuntu 18.04 (bionic beaver)
- Ubuntu 20.04 (focal fossa)

## Prerequisite
```bash
sudo apt install ansible
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

### Keybinds

#### emacs

C-; : completion

##### project
C-c p f : Find file in project

##### vcs
C-c v g : Magit status

##### file
C-c f r : Find recently opened file
