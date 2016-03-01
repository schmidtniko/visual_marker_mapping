# Install

## Cloning

git clone --recursive git@gitlab.uni-koblenz.de:ldebald/MarkerSlamRosFree.git



## Ceres

`sudo apt-get install libceres-dev libsuitesparse-dev`

## OpenCV 

'sudo apt-get install libopencv-dev'

## Qt5

`sudo apt-get install qt5-default`


## CMake command 

### Mac 
```
cmake -DCMAKE_PREFIX_PATH=<PATH_TO_QT5> -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
```
