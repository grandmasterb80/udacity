How to get the data for the tutorials

Preliminaries: YOLO
-------------------
Call following script:
  ../Autonomous_Vehicles_and_Computer_Vision/yolo/download.sh


Udacity: sensor fusion course
-----------------------------

SFND_Lidar_Obstacle_Detection:
------------------------------

Following directories are required from the original github repo:
 * SFND_Lidar_Obstacle_Detection/media
 * SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd

Required steps: clone following repository
git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git

The links to the data are already in the repository. There is no need to apply
any additional steps. Find below the links that were created for this particular
sub-project:
 * udacity/SFND_Lidar_Obstacle_Detection/media points to udacity/source/SFND_Lidar_Obstacle_Detection/media
 * udacity/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd points to udacity/source/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd


OpenCV - Midterm Project:
-------------------------

Following directories are required from the original github repo:
 * SFND_2D_Feature_Tracking/LICENSE
 * SFND_2D_Feature_Tracking/README.md
 * SFND_2D_Feature_Tracking/images

Required steps: clone following repository
git clone https://github.com/udacity/SFND_2D_Feature_Tracking.git

The links to the data are already in the repository. There is no need to apply
any additional steps. Find below the links that were created for this particular
sub-project:

Copy Media and PCD files into own copy of the repo
SFND_2D_Feature_Tracking/LICENSE points to ../source/SFND_2D_Feature_Tracking/LICENSE
SFND_2D_Feature_Tracking/README.md points to ../source/SFND_2D_Feature_Tracking/README.md
SFND_2D_Feature_Tracking/images points to ../source/SFND_2D_Feature_Tracking/images


OpenCV - Final Project:
-----------------------

Note: The final project requires YOLO. See instructions above.

git clone --recursive https://github.com/udacity/SFND_3D_Object_Tracking.git

Copy Media and PCD files into own copy of the repo
    Source: SFND_3D_Object_Tracking/LICENSE
    Source: SFND_3D_Object_Tracking/README.md
    Source: SFND_3D_Object_Tracking/images
    Source: SFND_3D_Object_Tracking/dat

Instructions:
    ln -s SFND_3D_Object_Tracking/images     ../SFND_3D_Object_Tracking/
    ln -s SFND_3D_Object_Tracking/dat        ../SFND_3D_Object_Tracking/
    ln -s SFND_3D_Object_Tracking/dat/yolo
    cd    SFND_3D_Object_Tracking/dat
    ln -s ../../Autonomous_Vehicles_and_Computer_Vision/yolo ./



SFND_Unscented_Kalman_Filter
----------------------------

git clone https://github.com/udacity/SFND_Unscented_Kalman_Filter

Copy Media and PCD files into own copy of the repo
    Source: SFND_Unscented_Kalman_Filter/README.md
    Source: SFND_Unscented_Kalman_Filter/media
    Source: SFND_Unscented_Kalman_Filter/src/sensors/data
    Source: SFND_Unscented_Kalman_Filter/src/Eigen

Instructions:
    mkdir -p ../SFND_Unscented_Kalman_Filter/src/sensors/data

    # only for the very initial check-out; afterwards .gitignore will be checked in
    echo "README.md" >> ../SFND_Unscented_Kalman_Filter/.gitignore
    echo "media" >> ../SFND_Unscented_Kalman_Filter/.gitignore
    echo "src/sensors/data" >> ../SFND_Unscented_Kalman_Filter/.gitignore
    echo "src/Eigen" >> ../SFND_Unscented_Kalman_Filter/.gitignore

    cp -a  SFND_Unscented_Kalman_Filter/README.md         ../SFND_Unscented_Kalman_Filter/
    cp -ra SFND_Unscented_Kalman_Filter/media             ../SFND_Unscented_Kalman_Filter/
    cp -ra SFND_Unscented_Kalman_Filter/src/sensors/data  ../SFND_Unscented_Kalman_Filter/src/sensors/
    cp -ra SFND_Unscented_Kalman_Filter/src/Eigen         ../SFND_Unscented_Kalman_Filter/src/
