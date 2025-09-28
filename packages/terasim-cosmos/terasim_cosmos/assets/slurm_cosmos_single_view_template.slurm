#!/bin/bash

#SBATCH --job-name=cosmos_single_view
#SBATCH --account=mcity_project
#SBATCH --partition=mcity_project
#SBATCH --time=0-01:00:00 # The format is Day-Hour:Minitue:Second
#SBATCH --nodes=1
#SBATCH --cpus-per-task=2 
#SBATCH --tasks-per-node=1
#SBATCH --gpus-per-node=h100:1 # e.g. h100:1 or l40s:2
#SBATCH --mem=128GB # e.g.128GB
#SBATCH --mail-type=END,FAIL 
#SBATCH --output=/home/jiawe/jobs/logs/cosmos-%x-%j.log 
#SBATCH --error=/home/jiawe/jobs/logs/cosmos-%x-%j.err

source /home/jiawe/anaconda3/etc/profile.d/conda.sh
conda activate cosmos-transfer1
cd /home/jiawe/cosmos-transfer1
bash {script_path}
