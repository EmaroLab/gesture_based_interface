# Gesture-based interface for Baxter Robot

## Download instructions
```
mkdir ~/sofar_ws
cd ~/sofar_ws
git clone https://github.com/EmaroLab/gesture_based_interface.git src
catkin_make
. devel/setup.bash
cd src
git status
```

## Kinect launcher
```
roscore
```
Launch nodes for the Kinect
```
roslaunch openni_launch openni.launch
```
Configuration: generation of environments according to the orientation angle of the Kinect
```
roslaunch pose_estimation config.launch
```
Estimation of the position of human's center of mass
```
roslaunch pose_estimation pose_estimator.launch
```
Service for setting the orientation angle of the Kinect: 
A node inside the package kinect_setup publishes the angle on the topic tilt_angle
```
rosservice call /move_kinect "angle: <float>"
```

```
rviz -> topic : /camera/pcl_filtered
	fixed frame: camera_link
	orbit: fixed frame (on the right)
```
