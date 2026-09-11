#!/usr/bin/env sh
set -eu

DOTFILES_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)

for target in "$HOME/CLAUDE.md" "$HOME/AGENTS.md" "$HOME/.codex/AGENTS.md"; do
    test -L "$target"
    cmp "$DOTFILES_DIR/prompts/home.md" "$target"
done

test -L "$HOME/.claude/settings.json"
cmp "$DOTFILES_DIR/roles/common/agents/settings.json" "$HOME/.claude/settings.json"
