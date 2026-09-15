# Debian/Ubuntu/macOS/Windows Settings for mqcmd196
![sudoer test](https://github.com/mqcmd196/dotfiles/actions/workflows/sudoer.yml/badge.svg?branch=master) ![non-sudoer test](https://github.com/mqcmd196/dotfiles/actions/workflows/non-sudoer.yml/badge.svg?branch=master)

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
### macOS (Apple Silicon and Intel)
Install [Homebrew](https://brew.sh/) first, then run:

```bash
brew install ansible git
cd ~/dotfiles
ansible-playbook setup_macos.yml
```

Homebrew packages are installed as your user, without `sudo`. An existing Emacs
installation is reused. The playbook links
shared zsh, tmux, Git, and agent settings and installs shell plugins. The same
role detects macOS when called through `setup_sudoer.yml`.

macOS uses the lightweight Emacs configuration (`non-sudoer/emacs.el`), since the
full configuration requires Debian-packaged Emacs extensions. Linux-only apt
repositories, ROS, Docker Engine, fonts, and Miniconda are not installed on macOS.
Install GUI apps and a conda distribution separately if needed. Terminal clipboard
copying in tmux uses `pbcopy`, and zsh loads Homebrew plugins from the appropriate
prefix. Existing conflicting configuration files may need to be backed up before
running the playbook.

To link only configuration files (without installing packages):

```bash
ansible-playbook setup_macos.yml --tags configs,claude,codex,prompts
```

### Debian / Ubuntu
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

C-u C-SPC : Back to the initial position when search

M-[ : Return to the beginning of the paragraph that precedes the point

M-] : Move to the end of the paragraph behind the point

M-h : Place a point and mark before or after the paragraph with the point or the paragraph following the point

C-x C-M-+, C-x C-M-=, C-x C-M-- or C-x C-M-0, or scroll the mouse wheel with both the Ctrl and Meta modifiers pressed : Change the sizes of the fonts globally

M-g i : imenu

C-x c i : helm-imenu

`lsp-workspace-restart` : Reload lsp at current workspace. It is convenient after the build

`reftex-toc` : Show the table of contents for the current tex document

##### project
C-c p f : Find file in project

##### vcs
C-c v g : Magit status

##### file
C-c f r : Find recently opened file

## CI
GitHub Actions tests the macOS Ansible setup on Apple Silicon (`macos-15`),
using an isolated home directory (see the
[GitHub runner reference](https://docs.github.com/en/actions/reference/runners/github-hosted-runners)). It checks repeat-run
idempotency, interactive zsh startup, Emacs loading, tmux clipboard bindings, and
shared agent links. The limited installer is tested on Ubuntu only; the existing
Debian/Ubuntu container matrix remains enabled.

After installing the dotfiles, run the configuration checks with:

```bash
sh tests/test.sh
```
