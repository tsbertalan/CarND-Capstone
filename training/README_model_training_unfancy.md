#### Environment Setup:
 - Ubuntu 16.04 LTS
 - CUDA 8.0 compatible videocard
 - Latest Nvidia Drivers (390)
  - Install by adding PPA
 - CUDA 8.0 and CuDNN 7.0 installed
  - ***TODO: Add Instruction***
 - Tensorflow 1.4.0 or Tensorflow-gpu 1.4.0
 ```
 pip install tensorflow==1.4.0
 ```
 - Python 2.7 and pip 8.1.1
 - Install dependencies protobuf-compiler, python-pil python-lxml python-tk
 ```
 sudo apt-get install protobuf-compiler python-pil python-lxml python-tk
 ```
 - download protoc-3.4.0 (linux) and extract to <protoc_root>
 ```
 https://github.com/protocolbuffers/protobuf/releases/tag/v3.4.0
 ```
 - clone Tensorflow model repository into <ts_models_root>
 ```
 git clone https://github.com/tensorflow/models.git
 ```
 - check out specific revision
 ```
 git checkout f7e99c0
 ```
 - go to <ts_models_root>/research/ and run:
 ```
 <protoc_root>/bin/protoc object_detection/protos/*.proto --python_out=.
 ```
 - add tensorflow models to environment variables
 ```
 export PYTHONPATH=$PYTHONPATH:/<ts_models_root>/models:/<ts_models_root>/models/research:/<ts_models_root>/models/research/slim:/<ts_models_root>/models/research/object_detection
 ```
 - create a project folder <project_folder> and the folders config, data, and models
 ```
 <project_folder>
 |
 └─config
 └─data
 └─models
 ```
 - copy `train.py` and `export_inference_graph from.py` from  `<ts_models_root>/models/research/object_detection` into `<project_folder>`
 - copy `ssd_inception_v2_coco.config` from `<ts_models_root>/models/research/object_detection/samples/configs` into `<project_folder>/config`
 - modify `ssd_inception_v2_coco.config`
  - ***TODO: .config file modifications***
 - download and extract COCO model into `<project_folder>/models`
 ```
 http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_11_06_2017.tar.gz
 ```
 - put dataset folder, \*.records, label_map.pbtxt into `<project_folder>/data`
  - ***TODO: create data set***

#### Train Model

- from `<project_folder>` run:
```
python train.py --logtostder --train_dir=./models/<training_folder> --pipeline_config_path=./config/ssd_inception_v2_coco.config
```

#### Save Frozen Graph

- Downgrade to Tensorflow 1.3.0
- from `<project_folder>` run:
```
python export_inference_graph.py --input_type image_tensor --pipeline_config_path ./config/ssd_inception_v2_coco.config --trained_checkpoint_prefix ./models/<training_folder>/model.ckpt-20000 --output_directory ./models/<training_folder>
```
