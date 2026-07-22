#!/usr/bin/env bash
set -euo pipefail

OPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(git -C "$OPS_DIR" rev-parse --show-toplevel)"
MESSAGE="${1:-ops: update coordination state}"

cd "$REPO_ROOT"
git add codex_ops

if git diff --cached --quiet -- codex_ops; then
  echo "codex_ops: nothing to commit"
else
  git commit -m "$MESSAGE" -- codex_ops
fi

if git remote get-url origin >/dev/null 2>&1; then
  branch="$(git branch --show-current)"
  if git rev-parse --abbrev-ref --symbolic-full-name '@{u}' >/dev/null 2>&1; then
    git push
  else
    git push -u origin "$branch"
  fi
else
  echo "codex_ops: no origin remote configured; commit kept local"
fi
