#!/usr/bin/env bash
# sync_repos.sh
# Bash script to commit and push to two remotes

set -e # Exit immediately on any *unexpected* command failure

# Commit message (default: "Auto commit")
MESSAGE="${1:-Auto commit}"

# Navigate to repo directory
cd "/home/fhn/Documents/Github/UWAM_PDM25/" || {
  echo "Failed to cd into repo directory"
  exit 1
}

# Figure out the current branch (e.g. main, dev, etc.)
BRANCH="$(git rev-parse --abbrev-ref HEAD)"

echo "Using branch: $BRANCH"

# Add all changes
git add -A

# Only commit if there are actually changes
if [[ -n "$(git status --porcelain)" ]]; then
  echo "Committing changes..."
  git commit -m "$MESSAGE"
else
  echo "No changes to commit, skipping commit."
fi

# Push to first remote (e.g. GitHub)
echo "Pushing to origin..."
git fetch origin
git push origin "$BRANCH"

# Push to second remote (e.g. GitLab)
echo "Pushing to gitlab (uwam)..."
git fetch uwam
git pull --rebase uwam "$BRANCH"
git push uwam "$BRANCH"

git status
