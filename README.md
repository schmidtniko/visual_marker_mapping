# Install

## Cloning

Via HTTPS:

`git clone --recursive https://github.com/cfneuhaus/visual_marker_mapping.git`

Via SSH:

`git clone --recursive git@github.com:cfneuhaus/visual_marker_mapping.git`

## Dependencies

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

`wget https://agas.uni-koblenz.de/data/datasets/visual_marker_mapping/calibration_room1.zip`
