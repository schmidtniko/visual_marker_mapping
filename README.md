# Overview
This software allows the accurate 3D reconstruction of configurations of markers from camera images. It uses AprilTags by Olson, that can simply be printed out and attached to walls or objects. To perform the reconstruction, you need a calibrated camera with a fixed focal length (no auto focus). The camera needs to be calibrated using the typical OpenCV camera calibration model. Given good marker planarity, accurate camera calibration, sharp high-resolution images, very high precision (<1-2mm) can be achieved.

Our software consists of two tools:
* *visual_marker_detection*: Performs marker detection on images in a selected folder and writes all detections to a JSON-file.
* *visual_marker_mapping*: Reads the marker detection file and performs the 3D reconstruction. The results of the reconstruction are again written to a JSON-file that contains the poses of the reconstructed cameras and markers.

![Overview of the Software](https://github.com/cfneuhaus/visual_marker_mapping/blob/master/doc/images/vmm1.png)

# Installation

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

## Cloning

Via HTTPS:

`git clone --recursive https://github.com/cfneuhaus/visual_marker_mapping.git`

Via SSH:

`git clone --recursive git@github.com:cfneuhaus/visual_marker_mapping.git`

## Building

You need at least
* [CMake 3.0](https://cmake.org/)
* GCC 4.7 (?)

If you are using Ubuntu, this means that you need at least Ubuntu 14.04.

### Linux/Mac

```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j5
```

### Windows

It should be possible to build our software on Windows, given that we do not use any platform specific features, but so far we have not attempted to build it there. Please let us know if you run into problems doing this.

# Usage

## Preliminaries

Our software works on *project paths*. A project path initially has to have the following layout:

```
my_project/.
my_project/camera_intrinsics.json
my_project/images/
my_project/images/your_image_1.jpg
my_project/images/anotherimage.png
...
```

* The *images* folder is supposed to contain all images that you want to use for calibration. Currently, all png and jpg files within the folder are being used. Note that they all have to have the same size, and they all have to correspond to the camera intrinsics specified in the *camera_intrinsics.json* file.
* The camera_intrinsics.json file is something you have to create before mapping (it is not required for detection only). See the [File Formats](#file-formats) section on how to create this one.
* Results of our tools are automatically written to the root of the project path. For example the marker detection writes a file called "marker_detections.json" to the root. The reconstruction result file is called "reconstruction.json".

## Running

Our software contains two command-line tools *visual_marker_detection* and *visual_marker_mapping*, both located in the *build/bin* folder, and a python script that can optionally be used to visualize the results in 3D.

visual_marker_detection:
* `--help`: Shows a help text.
* `--project-path`: Path to the aforementioned project directory.
* `--marker_width`, `--marker-height`: The marker width/height in meters. This is a marker size that is written to the *marker_detections.json* file with every marker. It is not used in any other way by the detection right now, but we feel that this information is an essential part of the detection result, which is why we include it. The marker can be configured to be slightly non-square, which can be used to compensate for bad and slightly distorted print-outs. If you have markers with different sizes, you will have to edit the *marker_detections.json* file by hand. If you do not care about the metrical size if your reconstruction, you can simply set both the width/height to any value, say *0.1*, or simply stick to the default value.
* `--do-corner-refinement`: We noticed that the AprilTags library we used, has issues with very high-resolution images, like DSLR images of slightly non-planar markers. To counteract this issue, we have added OpenCVs corner refinement to our software. We only recommend using flag in the described case though. For lower resolution images or planar markers, existing corner localization method is fine.
* `--marker_type`: Allows to configure the type of markers that are being searched in the images. The default value is "apriltag_36h11", but we also support: "apriltag_16h5", "apriltag_25h7", "apriltag_25h9" or "apriltag_36h9"
* **Returns**: While running, our tool will print the file it is currently working on, as well as the marker ids that have been detected in the respective image. Upon completion, the *marker_detections.json* file is written to the project path.

visual_marker_mapping:
* `--help`: Shows a help text.
* `--project-path`: Path to the aforementioned project directory.
* `--start-tag-id`: The id of a tag that should be used as the origin of the coordinate system. It is suggested to use a tag that is located on the ground as one of the start tags. If you do not specify a start tag, the software will chose one itself.
* **Returns**: Upon completion, the *reconstruction.json* file is written to the project path.

For visualization of the results in 3D, we also include a Python (2.7/3.0) script called "visualize_reconstruction.py". It is based on *pygame*, *OpenGL*, *GLU*, *GLUT*, *numpy*, and you may need to install the corresponding Python packages for your distribution in order to be able to run it.

The tool's only parameter is the path of the reconstruction.json file, that is being written by the visual_marker_mapping tool upon completion. The camera can be controlled using W, S, A, D. The mouse can be used to look around by holding the left mouse button. The camera speed can be increased by holding space.

## Example

We provide a test dataset, that you can use to test our tools. It is available [here](https://agas.uni-koblenz.de/data/datasets/visual_marker_mapping/calibration_room1.zip).

Use the following steps to perform the marker detection and 3D reconstruction (assuming you are in the root folder of this repository):

```
wget https://agas.uni-koblenz.de/data/datasets/visual_marker_mapping/calibration_room1.zip
unzip calibration_room1.zip
./build/bin/visual_marker_detection --project_path calibration_room1 --marker_width 0.1285 --marker_height 0.1295 --do_corner_refinement
./build/bin/visual_marker_mapping --project_path calibration_room1 --start_tag_id 230
```

If you want to visualize the results, simply run:
```
python3 visualize_reconstruction.py calibration_room1/reconstruction.json
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

Occurring rotations are represented as a unit quaternion in the order *w, x, y, z*. Rotation and translation together define a pose that transforms points from marker/camera space to world space. The local coordinate systems are defined as follows:
* When looking at a marker, the *x*-axis goes to the right, *y* up, and *z* points out of the marker plane.
* A cameras *x*-axis points to the right, *y*-axis down, and the *z*-axis in viewing direction.

# Authors

* Frank Neuhaus (fneuhaus_AT_uni-koblenz.de)
* Stephan Manthe
* Lukas Debald

# Copyright and License

[GPL 3.0](http://www.gnu.org/licenses/gpl-3.0.en.html)

# Citing

If you use our work, please cite us:

TODO

# References

Ed Olson, [AprilTag: A robust and flexible visual fiducial system](http://april.eecs.umich.edu/papers/details.php?name=olson2011tags), Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), 2011
