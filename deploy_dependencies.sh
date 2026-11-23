#!/bin/bash
set -euo pipefail

# Resolve source paths relative to this script, even when run from another folder.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
user="ubuntu"
host="${PI_HOST:-192.168.10.28}"
remote_dir="/home/ubuntu/lassie-turtle/dependencies"

echo "Deploying dependencies to $user@$host:$remote_dir"
ssh "$user@$host" "mkdir -p '$remote_dir'"

# Transfer source only. The Pi builds its own architecture-specific artifacts.
# Use normal SSH authentication (a configured key or a password).
rsync -azP \
    --exclude='build/' \
    --exclude='install/' \
    --exclude='log/' \
    "$script_dir/dependencies/" "$user@$host:$remote_dir/"

echo "Deployment complete. Build on the Pi in $remote_dir."
