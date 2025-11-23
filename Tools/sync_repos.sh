#!/usr/bin/env bash
# sync_repos.sh
# Bash/POSIX script to commit and push to two remotes

set -e # Exit on error

# Commit message (default: "Auto commit")
MESSAGE="${1:-Auto commit}"

# Navigate to repo directory
cd "/home/fhn/Documents/Github/UWAM_PDM25/" || {
  echo "Failed to cd into repo directory"
  exit 1
}

# Figure out current branch (e.g. main, dev)
BRANCH="$(git rev-parse --abbrev-ref HEAD)"
echo "Using branch: $BRANCH"

# Stage all changes
git add -A

# Only commit if there are changes (staged or unstaged)
if [ -n "$(git status --porcelain)" ]; then
  echo "Committing changes..."
  git commit -m "$MESSAGE"
else
  echo "No changes to commit, skipping commit."
fi

echo "Pushing to origin..."
git push origin "$BRANCH"

echo "Pushing to gitlab (uwam)..."
git pull --rebase uwam "$BRANCH"
git push uwam "$BRANCH"

git status
