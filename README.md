# Overview
This software allows the accurate 3D reconstruction of configurations of markers from camera images. It uses AprilTags by Olson, that can simply be printed out and attached to walls or objects. To perform the reconstruction, you need a calibrated camera with a fixed focal length (no auto focus). The camera needs to be calibrated using the typical OpenCV camera calibration model. Given good marker planarity, accurate camera calibration, sharp high-resolution images, very high precision (<1-2mm) can be achieved.

Our software consists of two tools:
* visual_marker_detection: Performs marker detection on a selected folder and writes all detections to a JSON-file.
* visual_marker_mapping: Reads the marker detection file and performs the 3D reconstruction. The results of the reconstruction are again written to a JSON-file that contains the poses of the reconstructed cameras and markers.

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

### Linux

```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j5
```

### Mac 

`cmake -DCMAKE_PREFIX_PATH=<PATH_TO_QT5> -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..`

# File Formats

## Camera Calibration File

Filename: `camera_intrinsics.json`
```json
{
    "fx" : "8.0752937867635346e+03",
    "fy" : "8.0831676114192869e+03",
    "cx" : "3.0163896805084278e+03",
    "cy" : "1.9962896554785455e+03",
    "distortion_coefficients" :
    [
       "-1.8618183262669760e-01", "3.7018092365577054e-01",
       "-2.9390604003594177e-04", "4.1533180829908799e-04",
       "5.7043887874185996e-02"
    ],
    "vertical_resolution" : "4000", 
    "horizontal_resolution" : "6000"
}
```

# Example

Test Dataset:

```
wget https://agas.uni-koblenz.de/data/datasets/visual_marker_mapping/calibration_room1.zip
unzip calibration_room1.zip
visual_marker_detection --project_path calibration_room1
visual_marker_mapping --project_path calibration_room1
```


# Copyright and License

TODO

# Citing

If you use our work, please cite us:

TODO

# References

Ed Olson, [AprilTag: A robust and flexible visual fiducial system](http://april.eecs.umich.edu/papers/details.php?name=olson2011tags), Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), 2011
