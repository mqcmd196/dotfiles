# Debian/Ubuntu Settings for mqcmd196
![sudoer test](https://github.com/mqcmd196/dotfiles/actions/workflows/sudoer.yml/badge.svg?branch=master) ![non-sudoer test](https://github.com/mqcmd196/dotfiles/actions/workflows/non-sudoer.yml/badge.svg?branch=master)
## Supported OS
- Debian 11 (bullseye)
- Ubuntu 18.04 (bionic beaver)
- Ubuntu 20.04 (focal fossa)

## Prerequisite
```bash
sudo apt install ansible make python3-pip python3-setuptools gnupg
```

## Setup
### When you are authorized to install package with sudo
```bash
make sudoer
```
