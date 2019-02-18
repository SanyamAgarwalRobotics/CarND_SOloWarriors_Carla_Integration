# Traffic light detection 



## Commands for training and exporting for inference
For copy and paste. :)

Needs dataset of TFRecord format and label map of pbtxt type.


```
# From tensorflow/models/research/
# Add Libraries to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
# Testing the Installation
python object_detection/builders/model_builder_test.py
```

#### faster_rcnn_resnet101_coco_2018_01_28
##### Train
```
# From tensorflow/models/research/
# for real data
python object_detection/train.py --logtostderr --pipeline_config_path config/real/faster_rcnn_resnet101_tl.config --train_dir train_dir/real/faster_rcnn_resnet101_coco_2018_01_28

# for simulator data
python object_detection/train.py --logtostderr --pipeline_config_path config/sim/faster_rcnn_resnet101_tl.config --train_dir train_dir/sim/faster_rcnn_resnet101_coco_2018_01_28
```
##### Export frozen models
```
# From tensorflow/models/research/
# for real data
python object_detection/export_inference_graph.py --pipeline_config_path config/real/faster_rcnn_resnet101_tl.config --trained_checkpoint_prefix train_dir/real/faster_rcnn_resnet101_coco_2018_01_28/model.ckpt-{#####} --output_directory frozen_models/real/faster_rcnn_resnet101_coco_2018_01_28

# for simulator data
python object_detection/export_inference_graph.py --pipeline_config_path config/sim/faster_rcnn_resnet101_tl.config --trained_checkpoint_prefix train_dir/sim/faster_rcnn_resnet101_coco_2018_01_28/model.ckpt-{#####} --output_directory frozen_models/sim/faster_rcnn_resnet101_coco_2018_01_28
```


### tensorboard
```
# From tensorflow/models/research/
export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
tensorboard --logdir=train_dir/real/faster_rcnn_resnet101_coco_2018_01_28
```
### Result from New FRCNN model

- Use below notebook for [Model Testing](https://github.com/SanyamAgarwalRobotics/CarND_SOloWarriors_Carla_Integration/blob/master/ros/src/tl_detector/light_classification/object_detection_udacity_real.ipynb)

- [Real Testing Images](https://github.com/SanyamAgarwalRobotics/CarND_SOloWarriors_Carla_Integration/blob/master/ros/src/tl_detector/light_classification/test_images_udacity)
