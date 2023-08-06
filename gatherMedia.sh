#!/bin/bash

# Define the folders to sync
remote_folder="/home/ubuntu/roboland/experiment_records/*"
local_folder="./experiment_records"

# SSH connection details
user="ubuntu"
host="192.168.8.100"

# Sync remote directory to the local directory
rsync -azP -e "ssh -i ~/.ssh/id_rsa" "$user@$host:$remote_folder" "$local_folder"
