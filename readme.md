# MayTag

This is a fully reworked version of of the [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag).


# Features
* с++
* header only
* generally faster than the original
* generally uses less memory than the original (especially for tags with a large amount bits and a correction errors)
* completely redesigned contouring algorithm
* other fixes and possible new bugs :)


# Examples
* [generator](example/generator) - tag image generator
* [camera](example/camera) - demonstrates MayTag work on data from the camera
* [apriltag](example/apriltag) - simultaneous work MayTag and original AprilTag on data from the camera
