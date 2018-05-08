# 1 Install Opencv
# 2 compile(Linux/Mac OS)
In Linux there may be some problem with the fisheye::stereoRectify
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
#3 Add your calibration parameters to stereo_camera.yml.
Attention: if you use matlab calibration tools, Just remember that the camera
intrinsic matrix in matlab is transpose of the normal camera intrinsic matrix.
Other parameters can be copied to stereo_camera.yml
#4 Run
```
./fisheye_rectify ../camear_stereo.yml ../images/l.png ../images/r.png
```

