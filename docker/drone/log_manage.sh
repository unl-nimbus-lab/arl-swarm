#!/bin/bash

timestamp=$(date +"%Y%m%d%H%M%S")
directory_name="veriflogs/veriflogs_$timestamp"

mkdir "$directory_name"
source CURRENT_LOG_DIR="$directory_name"
echo "Created directory: $directory_name"
