#!/bin/bash
# Check the status of the repository
git status

# Stage all changes for commit
git add .

# Prompt for commit message
echo "Enter commit message: "
read commit_message

# Commit the changes with the provided message
git commit -m "$commit_message"

# Push the changes to GitHub
git push origin struktur_oppdatering

