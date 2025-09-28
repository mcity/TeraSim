#!/bin/bash
eval "$(conda shell.bash hook)"
conda activate cosmos-transfer1

export PROMPT="{prompt}"
export CUDA_VISIBLE_DEVICES="${{CUDA_VISIBLE_DEVICES:=0}}"
export CHECKPOINT_DIR="${{CHECKPOINT_DIR:=/scratch/mcity_project_root/mcity_project/jiawe/cosmos-transfer1-0624/checkpoints}}"
export NUM_GPUS=1
PYTHONPATH=$(pwd) torchrun --nproc_per_node=${{NUM_GPUS}} cosmos_transfer1/diffusion/inference/transfer_multiview.py \
    --seed 1996 \
    --checkpoint_dir $CHECKPOINT_DIR \
    --video_save_name {video_name} \
    --video_save_folder {video_folder} \
    --offload_text_encoder_model \
    --guidance 3 \
    --controlnet_specs {controlnet_spec} \
    --num_gpus ${{NUM_GPUS}} \
    --num_steps 30 \
    --view_condition_video {view_condition_video} \
    --prompt "$PROMPT"