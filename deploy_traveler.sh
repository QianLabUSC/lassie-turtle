#!/bin/bash

# Define the folders to copy
folders=("docker" "highlevel")

# SSH connection details
user="ubuntu"
host="192.168.8.100"
remote_dir="/home/ubuntu/roboland"


# Sync folders to the remote directory
for folder in "${folders[@]}"; do
    rsync -azP -e "ssh -i ~/.ssh/id_rsa" "$folder" "$user@$host:$remote_dir"
done
