#!/bin/bash

wget https://pjreddie.com/media/files/yolov3.weights
wget https://pjreddie.com/media/files/yolov3-tiny.weights

git clone -n https://github.com/pjreddie/darknet.git --depth 1
cd darknet/
git checkout HEAD cfg/yolov3.cfg
git checkout HEAD cfg/yolov3-tiny.cfg
git checkout HEAD data/coco.names

cd ..

ln -s darknet/cfg/yolov3.cfg
ln -s darknet/cfg/yolov3-tiny.cfg
ln -s darknet/data/coco.names

#wget https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg
#wget https://github.com/tahaemara/yolo-custom-object-detector/blob/afef9e2ab1fc91212a0206df3a06d23f4792e164/python/custom/yolov3-tiny.cfg

