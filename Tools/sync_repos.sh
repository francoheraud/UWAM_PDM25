#!/usr/bin/env bash
# sync_repos.sh
# Bash script to commit and push to two remotes

set -e # Exit on error

# Commit message (default: "Auto commit")
MESSAGE="${1:-Auto commit}"

# Navigate to repo directory
cd "/home/fhn/Documents/Github/UWAM_PDM25/" || {
  echo "Failed to cd into repo directory"
  exit 1
}

# Local branch (current branch)
BRANCH="$(git rev-parse --abbrev-ref HEAD)"
echo "Using local branch: $BRANCH"

# Remote branch on UWAM (GitLab)
UWAM_BRANCH="master"

# Stage all changes
git add -A

# Only commit if there are changes
if [ -n "$(git status --porcelain)" ]; then
  echo "Committing changes..."
  git commit -m "$MESSAGE"
else
  echo "No changes to commit, skipping commit."
fi

echo "Pushing to origin (GitHub)..."
git push origin "$BRANCH"

echo "Pushing to uwam (GitLab master)..."
git push uwam "$BRANCH:$UWAM_BRANCH"

git status
