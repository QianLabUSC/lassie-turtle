#!/bin/bash

# Replace these variables with your actual SSH connection details
remote_user="ubuntu"
remote_host="${PI_HOST:-192.168.10.28}"

echo "Attempting to establish SSH connection to $remote_user@$remote_host..."

# Loop until the SSH connection is successful
while true; do
    ssh -o "ConnectTimeout=5" $remote_user@$remote_host
    if [ $? -eq 0 ]; then
        echo "SSH connection established successfully!"
        break
    else
        echo "SSH connection not yet available. Retrying in 5 seconds..."
        sleep 5
    fi
done



