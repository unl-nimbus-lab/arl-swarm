#!/bin/bash

timestamp=$(date +"%Y")"-"$(date +"%m")"-"$(date +"%d")"-"$(date +"%H")"-"$(date +"%M")"-"$(date +"%S")
directory_name=$HOME"verifier_logs/verifier_logs_$timestamp"

mkdir "$directory_name"
export CURRENT_LOG_DIR="$directory_name"
echo "Created directory: $directory_name"