# CameraLidarCalibrator
A package for spatiotemporal calibration of camera and 3D laser scanner using a typical chessboard pattern.

# Current state

I wanted to refactor the original code to make it more readable and easy to use. Unfortunately, it is not yet finished.
I will try to:
* provide more examples
* make automatic detection work on LiDAR data (it is not yet finished)
* adapt scripts I did to perform statistical analysis on simulation

# How to build

Build g2o:

``
bash buildThirdparty.sh
``

Then you can compile the package using ROS melodic:

``
clear && catkin build --cmake-args  -DCMAKE_BUILD_TYPE=Release --make-args -j 3
``

# Running real-data examples

First download the data by typing:

``
bash prepareExemplaryData.sh
``

Then you can run real-date calibration (based on information in ``launch/run_rosbag_velo.launch``) with:

``
roslaunch camera_lidar_calibrator run_rosbag_velo.launch  
``

The calibration is run on preprocessed image and point cloud detections. If you want to redo camera detection set 
``read_chessboards_from_file`` to false in launch file and the image will be processed at the beginning of the calibration.
If you want to redo laser detections set ``read_countours_from_file`` to false in launch file and you will be asked to manually
mark 4 corners of the chessboard based on depth image (automatic detection is not reliable enough for now).

Some debugging information is stored in directory ``log``.

# Running simulation

You can run simulation (based on information in ``launch/run_sim.launch``) with:

``
roslaunch camera_lidar_calibrator run_sin.launch  
``

The code runs the simulation once. I will provide my scripts to automate simulation soon.

Some debugging information is stored in directory ``log``.

# Note
If you found the article/code useful, please cite work ([IEEE Explore](https://ieeexplore.ieee.org/document/9161262),
                                                        [arXiv](https://arxiv.org/abs/2006.16081)):

``
M. Nowicki, "Spatiotemporal Calibration of Camera and 3D Laser Scanner," 
in IEEE Robotics and Automation Letters, 
vol. 5, no. 4, pp. 6451-6458, Oct. 2020, doi: 10.1109/LRA.2020.3014639.
``

```
@ARTICLE{9161262,
  author={M. {Nowicki}},
  journal={IEEE Robotics and Automation Letters}, 
  title={Spatiotemporal Calibration of Camera and 3D Laser Scanner}, 
  year={2020},
  volume={5},
  number={4},
  pages={6451-6458}
```

# FAQ

##### 1. What are the requirements of your method?

Chessboard pattern (5 by 8, 10 cm grid is asssumed for now), global-shutter camera, shared view between the camera/3D LiDAR

##### 2. How can I use the laser point timestamps directly from the LiDAR driver?

You need to scale the timestamp from 0 (the beggining of the scan) to 1 (the end of the scan) and put the resulting value
into the 'intensity' part of the PointXYZI

##### 3. Is it possible to use spherical S3 inteprolation?

One of the reviewers asked for it, we tested and the system achieved the same results as for the truncated (2-dimensional) SO(3).
I just used SO(3) due to previous experience. 

##### 4. Do you use the borders of the chessboard pattern duing optimization?

No, I tried different ways of defining optimized error (like mixed point-to-plane and point-to-border) but the obtained 
results were usually worse while making it more complex (especially Jacobians) and more time-consuming.

##### 5. I cannot run the software and I need help

Please create a GitHub issue and attach the data you wanted to use during calibration. I will check it out.

##### 6. How do you perform chessboard detection/tracking?

For images, the typical chessboard detection is used. For pointclouds, you can either:

a) Mark is manually by 4 corners

b) Load manual markings from file

c) Try to track first manual marking using prepared tracking procedures

For best results, we used method 'a' (or 'b' to perform it multiple times). Treat 'c' as experimental work that has not 
yet been finished but looks promising. 

##### 7. Does your software provide also camera calibration?

No. The camera is assumed to be calibrated. It is possible to solve it jointly with the camera-laser calibration but it would 
make system more complex and I saw no need due to the abundance of available software for camera calibration (i.e. kalibr).

# License

We make our code available using BSD-3 license. Please note that the system requires g2o library ([github](https://github.com/RainerKuemmerle/g2o))
that has its own license. We include g2o library as a whole with non-BSD parts (directory EXTERNAL), although our code should work
based solely on BSD components.

CameraLidarCalibratior is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 