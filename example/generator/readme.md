# Generator

Tag image generator.


# Requirements
* OpenCV


# Usage

```
./maytag-generator -f=tag16h5 -b=1 -s=1
```

Arguments:
* -f - tag family (tag16h5, tag25h9, tag36h10, tag36h11)
* -b - tag color (1 - black, 0 - white)
* -s - bit size in pixel (> 0)

The generated tags will be located in the directory `'tags/<family>-<b/w>-<size>'`. For example: `'tags/tag16h5-b-1/'`.

