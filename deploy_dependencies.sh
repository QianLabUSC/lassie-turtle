#!/bin/bash

# Define the folders to copy
folders=("dependencies")

# SSH connection details
user="ubuntu"
host="192.168.10.20"
remote_dir="/home/ubuntu/roboland"

# Sync folders to the remote directory
for folder in "${folders[@]}"; do
    rsync -azP "$folder" "$user@$host:$remote_dir"
done
