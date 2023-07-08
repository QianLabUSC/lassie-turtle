#!/bin/bash

# Define the folders to copy
folders=("docker" "highlevel" "src")

# SSH connection details
user="travelerleg"
host="ubuntu2004"
remote_dir="/home/travelerleg"

# Delete existing folders in the remote directory
for folder in "${folders[@]}"; do
    ssh "$user@$host" "rm -r $remote_dir/$folder"
done

# Copy folders to the remote directory
for folder in "${folders[@]}"; do
    scp -i ~/.ssh/id_rsa -r "$folder" "$user@$host:$remote_dir"
done

