cd /home/data/Project/Deploy
python convert2lerobot.py \
    --raw-root /home/data/Dataset/rheovla_dataset_raw \
    --output-path /home/data/Dataset/rheovla_dataset_lerobot \
    --repo-id rheovla_dataset_lerobot \
    --fps 10 \
    --action-chunk-size 10 \
    --overwrite 
