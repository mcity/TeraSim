# Processing Waymo dataset for Cosmos-Predict1-SampleAV

## Installation
If you haven't set up the data processing environment yet, follow our installation instructions [here](https://github.com/nv-tlabs/cosmos-av-sample-toolkits?tab=readme-ov-file#installation) to setup the environment. 

## PostTraining

We provide a conversion and rendering script for the Waymo Open Dataset as an example of how information from another AV source can interface with the model. Note that our model is not trained on the Waymo dataset, and this script is intended to help users better understand our data format. As a result, a drop in generative video quality is expected. Finetuning on the desired custom dataset would be beneficial to improve quality.

### Waymo Open Dataset
Parsing tfrecords from Waymo Open Dataset requires extra dependencies; install it with
```bash
pip install waymo-open-dataset-tf-2-11-0==1.6.1
```

#### Step 0: Check Our Provided Captions
We provide auto-generated captions for the Waymo dataset at [`assets/waymo_caption.csv`](./assets/waymo_caption.csv). You will need these captions to run [**Cosmos-Transfer1-7B-Sample-AV**](https://github.com/nvidia-cosmos/cosmos-transfer1/blob/main/examples/inference_cosmos_transfer1_7b_sample_av.md).

#### Step 1: Download Waymo Open Dataset

Download the all the training & validation clips from [waymo perception dataset v1.4.2](https://waymo.com/open/download/) to the `<WAYMO_TFRECORDS_FOLDER>`. 

If you have `sudo`, you can use [gcloud](https://cloud.google.com/storage/docs/discover-object-storage-gcloud) to download them from terminal.
<details>
<summary><span style="font-weight: bold;">gcloud installation (need sudo) and downloading from terminal</span></summary>

```bash
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates gnupg curl
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo gpg --dearmor -o /usr/share/keyrings/cloud.google.gpg
echo "deb [signed-by=/usr/share/keyrings/cloud.google.gpg] https://packages.cloud.google.com/apt cloud-sdk main" | sudo tee -a /etc/apt/sources.list.d/google-cloud-sdk.list
sudo apt-get update && sudo apt-get install google-cloud-cli
```

Then you can login your google account and download the above tfrecords via
```bash
# or use `gcloud init --no-launch-browser` if you are in a remote terminal session
gcloud init 
bash download_waymo.sh config/waymo_all.json <WAYMO_TFRECORDS_FOLDER>
```
</details>

After downloading tfrecord files, we expect a folder structure as follows:
```bash
<WAYMO_TFRECORDS_FOLDER>
|-- segment-10247954040621004675_2180_000_2200_000_with_camera_labels.tfrecord
|-- segment-11379226583756500423_6230_810_6250_810_with_camera_labels.tfrecord
|-- ...
`-- segment-1172406780360799916_1660_000_1680_000_with_camera_labels.tfrecord
```

> [!NOTE]
> If you download the tfrecord files from the console, you will have prefixes like `individual_files_training_` or `individual_files_validation_`. Make sure these prefixes are removed before further processing.


#### Step 2: Convert Waymo Open Dataset to RDS-HQ format

Next, convert the Waymo Open Dataset to RDS-HQ format. Suppose you have a folder with Waymo Open Dataset's tfrecords, you can convert it to RDS-HQ format by:
```bash
python convert_waymo_to_rds_hq.py -i <WAYMO_TFRECORDS_FOLDER> -o <WAYMO_RDS-HQ_FOLDER>/videos -n 16
```
Here `<WAYMO_RDS-HQ_FOLDER>` can be set to any folder you want, and number of workers can be changed from `16`.  

#### Step 3: Create T5 Text Embeddings
Lastly, we need to create T5 text embeddings. 
Make sure you have completed Cosmos-predict1 installation and use the cosmos-predict1 environment for this step:
```bash
conda activate cosmos-predict1
```
We offer two set of captions, a more complete set of single view captions in `assets/waymo_caption.csv`, and a set of 5k multiview captions in `assets/waymo_multiview_texts.json`.
To use the 5k multiview captions in `assets/waymo_multiview_texts.json`:
```bash
python create_t5_embed.py --text_file ./assets/waymo_multiview_texts.json --data_root <WAYMO_RDS-HQ_FOLDER>
```
Alternatively, to use the single view captions in `assets/waymo_caption.csv`:
```bash
python create_t5_embed.py --caption_file ./assets/waymo_caption.csv --data_root <WAYMO_RDS-HQ_FOLDER>
```

The resulting folder structure should look like this:
```
<WAYMO_RDS-HQ_FOLDER>/waymo/
├── cache/
│   ├── prefix_t5_embeddings_pinhole_front.pickle
│   ├── prefix_t5_embeddings_pinhole_front_left.pickle
│   ├── prefix_t5_embeddings_pinhole_front_right.pickle
│   ├── prefix_t5_embeddings_pinhole_side_left.pickle
│   └── prefix_t5_embeddings_pinhole_side_right.pickle
├── videos/
│   ├── pinhole_front
│       ├── *.mp4
│   ├── pinhole_front_left
│   ├── pinhole_front_right
│   ├── pinhole_side_left
│   ├── pinhole_side_right
│   ...
└── t5_xxl/
    ├── pinhole_front
        └── *.pkl
```
You are now ready to train cosmos-predict1 models on Waymo!
