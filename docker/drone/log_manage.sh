#!/bin/bash                                                            
timestamp=$(date +"%Y")"-"$(date +"%m")"-"$(date +"%d")"-"$(date +"%H")"-"$(date +"%M")"-"$(date +"%S")

container_directory_name=$HOME"hosts_home/ubuntu/verifier_logs/verifier_logs_$timestamp"
host_directory_name=$HOME"ubuntu/verifier_logs/verifier_logs_$timestamp"

container_current_bag=$HOME"hosts_home/ubuntu/rosbags/"$timestamp.bag
host_current_bag=$HOME"ubuntu/rosbags/"$timestamp.bag

mkdir "$container_directory_name"
#export CURRENT_LOG_DIR="$directory_name"

export LATEST_BAG_CON=$container_current_bag
export LATEST_VER_CON=$container_directory_name

sed -i '/export LATEST_VER_CON=/d' /home/hosts_home/ubuntu/.bashrc
sed -i '/export LATEST_BAG_CON=/d' /home/hosts_home/ubuntu/.bashrc
sed -i '/export LATEST_VER_HOST=/d' /home/hosts_home/ubuntu/.bashrc
sed -i '/export LATEST_BAG_HOST=/d' /home/hosts_home/ubuntu/.bashrc

echo 'export LATEST_VER_HOST='$host_directory_name >> /home/hosts_home/ubuntu/.bashrc
echo 'export LATEST_BAG_HOST='$host_current_bag >> /home/hosts_home/ubuntu/.bashrc
echo 'export LATEST_VER_CON='$container_directory_name >> /home/hosts_home/ubuntu/.bashrc
echo 'export LATEST_BAG_CON='$container_current_bag >> /home/hosts_home/ubuntu/.bashrc
