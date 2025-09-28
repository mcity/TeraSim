#!/bin/bash
eval "$(conda shell.bash hook)"
conda activate cosmos-transfer1

export PROMPT="{prompt}"
export CUDA_VISIBLE_DEVICES=0
export RANK=0
export CHECKPOINT_DIR="${{CHECKPOINT_DIR:=/scratch/mcity_project_root/mcity_project/jiawe/cosmos-transfer1-0624/checkpoints}}"
PYTHONPATH=$(pwd) python cosmos_transfer1/diffusion/inference/transfer.py \
    --seed 1996 \
    --checkpoint_dir $CHECKPOINT_DIR \
    --video_save_name {video_name} \
    --video_save_folder {video_folder} \
    --prompt "$PROMPT" \
    --sigma_max 80 \
    --offload_text_encoder_model --is_av_sample \
    --controlnet_specs {controlnet_spec}
