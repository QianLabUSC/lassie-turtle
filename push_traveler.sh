#!/bin/bash
set -e
echo "Please provide a commit message for this commit of all repos: "
read commit_message
git add .
git commit -m "$commit_message"
git push
git submodule foreach "git add ."
git submodule foreach "git commit -m '$commit_message' || :"
git submodule foreach git push