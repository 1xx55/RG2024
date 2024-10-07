#!/bin/bash

record_time=15 # set time

# Create a new folder with timestamp
folder_name=$(date +%Y%m%d_%H%M%S) # e.g., 20240215_123456
mkdir -p "/home/nuc/Documents/mmwave/captures/$folder_name"

# Update filename to include the new folder path
filename="/home/nuc/Documents/mmwave/captures/$folder_name/capture.pcap" # tcpdump_save

source /home/nuc/miniconda3/etc/profile.d/conda.sh
conda activate mmwave

# run python script
python /home/nuc/Documents/mmwave/feb12-main/test_mmwave.py &

# run tcpdump and save to the new folder
sudo timeout ${record_time} tcpdump -i enp89s0 -w ${filename} -B 524288

# optional: deactivate the conda environment
conda deactivate
