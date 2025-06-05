#!/usr/bin/env bash

set -e

DOOM=~/.emacs.d/bin/doom
${DOOM} doctor --pager cat # check doom emacs config
