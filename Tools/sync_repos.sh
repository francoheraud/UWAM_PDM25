#!/usr/bin/env bash
# sync_repos.sh
# Bash script to commit and push to GitHub (main) and GitLab (master)

set -e # Exit on error

# Commit message (default: "Auto commit")
MESSAGE="${1:-Auto commit}"

# Navigate to repo directory
cd "/home/fhn/Documents/Github/UWAM_PDM25/" || {
  echo "Failed to cd into repo directory"
  exit 1
}

# Hardcode local + remote branches so Git stops being clever
LOCAL_BRANCH="main"
UWAM_BRANCH="master"

# Make sure we're on main
git checkout "$LOCAL_BRANCH"

echo "Using local branch: $LOCAL_BRANCH (-> origin/$LOCAL_BRANCH, uwam/$UWAM_BRANCH)"

# Stage all changes
git add -A

# Only commit if there are changes
if [ -n "$(git status --porcelain)" ]; then
  echo "Committing changes..."
  git commit -m "$MESSAGE"
else
  echo "No changes to commit, skipping commit."
fi

echo "Pushing to origin (GitHub, main)..."
git push origin "$LOCAL_BRANCH"

echo "Force-pushing to uwam (GitLab, master)..."
git push --force uwam "$LOCAL_BRANCH:$UWAM_BRANCH"

git status
