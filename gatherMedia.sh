#!/bin/bash

# Define the folders to sync
remote_folder="/home/ubuntu/roboland/experiment_records/MH23_Data/Friday8_11/*"
local_folder="./experiment_records/MH23_Data/Friday8_11/"

# SSH connection details
user="ubuntu"
host="192.168.2.1"

# Sync remote directory to the local directory
rsync -azP -e "ssh -i ~/.ssh/id_rsa" "$user@$host:$remote_folder" "$local_folder"
