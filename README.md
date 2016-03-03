# Overview
This software allows the 3D reconstruction of configurations of markers from camera images. It uses AprilTags by Olson, that can simply be printed out and attached to walls or objects. 

![Overview of the Software](https://github.com/cfneuhaus/visual_marker_mapping/blob/master/doc/images/vmm1.png)

# Installation

## Cloning

Via HTTPS:

`git clone --recursive https://github.com/cfneuhaus/visual_marker_mapping.git`

Via SSH:

`git clone --recursive git@github.com:cfneuhaus/visual_marker_mapping.git`

## Dependencies

* ceres
* opencv
* eigen

In Ubuntu, these dependencies can be installed using the command

`sudo apt-get install libceres-dev libsuitesparse-dev libopencv-dev`

## Building

### Ubuntu

```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j5
```

### Mac 

`cmake -DCMAKE_PREFIX_PATH=<PATH_TO_QT5> -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..`

# Testing

Test Dataset:

```
wget https://agas.uni-koblenz.de/data/datasets/visual_marker_mapping/calibration_room1.zip
unzip calibration_room1.zip
visual_marker_detection --project_path calibration_room1
visual_marker_mapping --project_path calibration_room1
```


# Copyright and License

TODO

# Publication

Ed Olson, [AprilTag: A robust and flexible visual fiducial system](http://april.eecs.umich.edu/papers/details.php?name=olson2011tags), Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), 2011
