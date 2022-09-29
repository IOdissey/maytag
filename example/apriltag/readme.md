# Apriltag example

Simultaneous work MayTag and original AprilTag on data from the camera.


# Requirements
* [OpenCV](https://github.com/opencv/opencv)
* [AprilTag](https://github.com/AprilRobotics/apriltag)


# Build
Standard cmake build.
```
mkdir build
cd build
cmake <path to CMakeLists.txt>
make
```


# Usage

```
./maytag-apriltag -d=0 -f=tag16h5 -b=1 -x=1.5
```

Arguments:
* `-h` - help
* `-d` - camera device number (default 0)
* `-iw` - image width (optionaly)
* `-ih` - image height (optionaly)
* `-f` - tag family: tag16h5, tag25h9, tag36h10, tag36h11 (default tag16h5)
* `-b` - tag color: 1 - black, 0 - white (default 1)
* `-ha` - number of error correction bits (hamming distance) (default 0)
* `-x` - decimate input image by this factor (supported 1.5, 2, 3, ...) (default 1)
* `-r` - spend more time trying to align edges of tags: 1 - on, 0 - off (default 1)