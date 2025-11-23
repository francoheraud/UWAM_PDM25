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

# Remote branch for uwam (set this to what you saw from git ls-remote)
UWAM_BRANCH="$BRANCH" # change to "master" or whatever if needed

# Stage all changes
git add -A

# Only commit if there are changes
if [ -n "$(git status --porcelain)" ]; then
  echo "Committing changes..."
  git commit -m "$MESSAGE"
else
  echo "No changes to commit, skipping commit."
fi

echo "Pushing to origin..."
git push origin "$BRANCH"

echo "Pushing to gitlab (uwam)..."

# Only pull --rebase if the branch exists on uwam
if git ls-remote --exit-code --heads uwam "$UWAM_BRANCH" >/dev/null 2>&1; then
  echo "Remote branch '$UWAM_BRANCH' exists on uwam, pulling with rebase..."
  git pull --rebase uwam "$UWAM_BRANCH"
else
  echo "Remote branch '$UWAM_BRANCH' does not exist on uwam, skipping pull (will create on push)."
fi

# Push local branch to that branch name on uwam
git push uwam "$BRANCH:$UWAM_BRANCH"

git status
