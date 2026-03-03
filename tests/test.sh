#!/usr/bin/env sh
set -e
DIR=$(dirname "$0")
for file in "$DIR"/test_*.sh; do
    sh "$file"
done
