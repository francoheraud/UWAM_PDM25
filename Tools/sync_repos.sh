#!/usr/bin/env bash
# sync_repos.sh
# Bash script to commit and push to two remotes

set -e # Exit immediately on any command failure

# Commit message (default: "Auto commit")
MESSAGE="${1:-Auto commit}"

# Navigate to repo directory (change this path for your system)
cd "/home/fhn/Documents/Github/UWAM_PDM25/" 2>/dev/null

# Add all changes
git add -A

# Commit with a message
git commit -m "$MESSAGE"

# Push to first remote (e.g. GitHub)
echo "Pushing to origin..."
git fetch origin
git push --force origin main

# Push to second remote (e.g. GitLab)
echo "Pushing to gitlab..."
git fetch uwam
git pull --rebase uwam main
git push --force uwam main

git status
