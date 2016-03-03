# Overview
This software allows the accurate 3D reconstruction of configurations of markers from camera images. It uses AprilTags by Olson, that can simply be printed out and attached to walls or objects. To perform the reconstruction, you need a calibrated camera with a fixed focal length (no auto focus). The camera needs to be calibrated using the typical OpenCV camera calibration model. Given good marker planarity, accurate camera calibration, sharp high-resolution images, very high precision (<1-2mm) can be achieved.

Our software consists of two tools:
* *visual_marker_detection*: Performs marker detection on images in a selected folder and writes all detections to a JSON-file.
* *visual_marker_mapping*: Reads the marker detection file and performs the 3D reconstruction. The results of the reconstruction are again written to a JSON-file that contains the poses of the reconstructed cameras and markers.

![Overview of the Software](https://github.com/cfneuhaus/visual_marker_mapping/blob/master/doc/images/vmm1.png)

# Installation

## Cloning

Via HTTPS:

`git clone --recursive https://github.com/cfneuhaus/visual_marker_mapping.git`

Via SSH:

`git clone --recursive git@github.com:cfneuhaus/visual_marker_mapping.git`

## Dependencies

* [Ceres Solver](http://ceres-solver.org/)
* [OpenCV](http://opencv.org/)
* [Eigen 3.0](http://eigen.tuxfamily.org/)
* [AprilTags implementation](https://github.com/cfneuhaus/fast_mit_apriltags)

In Ubuntu, the first three dependencies can be installed using the command

* `apt-get install libceres-dev libsuitesparse-dev libopencv-dev`

In Arch Linux, use:

* `pacman -S eigen opencv`

* Ceres is available as an AUR package called [ceres-solver](https://aur.archlinux.org/packages/ceres-solver/).

The AprilTags dependency is automatically pulled in as a git submodule.

## Building

You need at least
* [CMake 3.0](https://cmake.org/)
* GCC 4.7 (?)

If you are using Ubuntu, this means that you need at least Ubuntu 14.04.

### Linux

```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j5
```

### Mac 

`cmake -DCMAKE_PREFIX_PATH=<PATH_TO_QT5> -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..`

### Windows

TODO

# Usage

TODO

## Example

Test Dataset:

```
wget https://agas.uni-koblenz.de/data/datasets/visual_marker_mapping/calibration_room1.zip
unzip calibration_room1.zip
visual_marker_detection --project_path calibration_room1
visual_marker_mapping --project_path calibration_room1
```

# File Formats

## Camera Calibration File

Filename: `camera_intrinsics.json`
```
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

Camera calibration model as defined [here](http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html):
* *fx*, *fy*: Focal length in px
* *cx*, *cy*: Principal point in px
* *distortion_coefficients*: Exactly 5 radial distortion coefficients, that are in the order *k_1, k_2, p_1, p_2, k_3*. Not required coefficients can be set to *0*.

## Marker Detections File

Filename: `marker_detections.json`
```
{
    "images":
    [
        {
            "filename": "DSC05028.JPG",
            "id": "0"
        },
        {
            "filename": "DSC05076.JPG",
            "id": "1"
        },
        [...]
    ],
    "tags":
    [
        {
            "id": "0",
            "tag_type": "apriltag_36h11",
            "width": "0.11650000000000001",
            "height": "0.11650000000000001"
        },
        {
            "id": "1",
            "tag_type": "apriltag_36h11",
            "width": "0.11650000000000001",
            "height": "0.11650000000000001"
        },
        [...]
    ],
    "tag_observations":
    [
        {
            "image_id": "0",
            "tag_id": "137",
            "observations":
            [
                [
                    "4753.32275390625",
                    "572.0101318359375"
                ],
                [
                    "5131.30810546875",
                    "568.25885009765625"
                ],
                [
                    "5120.79541015625",
                    "201.8472900390625"
                ],
                [
                    "4744.53369140625",
                    "205.41494750976562"
                ]
            ]
        },
        [...]
    ]
}
```

The individual marker corner observations are in the order: *Lower left, Lower right, Upper right, Upper left*

## Reconstruction Result File

Filename: `reconstruction.json`
```
{
    "reconstructed_tags":
    [
        {
            "id": "0",
            "type": "apriltag_36h11",
            "width": "0.11650000000000001",
            "height": "0.11650000000000001",
            "rotation":
            [
                "0.99998768285276463",
                "-0.0026651883409995361",
                "-1.5875056612292212e-05",
                "0.0041869633189374018"
            ],
            "translation":
            [
                "0.0061616789246268563",
                "0.37117687683311673",
                "-0.0027071129933752556"
            ]
        },
        [...]
    ],
    "reconstructed_cameras":
    [
        {
            "id": "0",
            "rotation":
            [
                "0.0071432235474879913",
                "0.048540245468997004",
                "0.050213553333055848",
                "0.9975326651237153"
            ],
            "translation":
            [
                "1.6566629838776454",
                "-1.0628296493529241",
                "-0.5984995791803972"
            ]
        },
        [...]
    ]
}
```

Occurring rotations are represented as a unit quaternion in the order *w,x,y,z*. Rotation and translation together define a pose that transforms points from marker/camera space to world space. The local coordinate systems are defined as follows:
* When looking at a marker, the *x*-axis goes to the right, *y* up, and *z* points out of the marker plane.
* A cameras *x*-axis points to the right, *y*-axis down, and the *z*-axis in viewing direction.

# Copyright and License

TODO

# Citing

If you use our work, please cite us:

TODO

# References

Ed Olson, [AprilTag: A robust and flexible visual fiducial system](http://april.eecs.umich.edu/papers/details.php?name=olson2011tags), Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), 2011
