cd /home/data/Project/Deploy
python convert2lerobot.py \
    --raw-root /home/data/Dataset/piper_dataset_raw \
    --output-path /home/data/Dataset/dynvla_dataset_lerobot \
    --repo-id dynvla_dataset_lerobot \
    --fps 10 \
    --overwrite 

#from huggingface_hub import HfApi
#HfApi().create_tag("trantor2nd/dynvla_dataset_lerobot",tag="v3.0",repo_type="dataset")