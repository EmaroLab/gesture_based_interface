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
roslaunch openni_launch openni.launch device_id:= A00362A07684107A
```
Configuration: generation of environments according to the orientation angle of the Kinect
```
roslaunch kinect_pcl_tools configuration.launch
```
Estimation of the position of human's center of mass
```
roslaunch kinect_pcl_tools kinect_to_fsm.launch
```
Service for setting the orientation angle of the Kinect: 
A node inside the package kinect_setup publishes the angle on the topic tilt_angle
```
rosservice call /move_kinect "angle: <float>"
```

```
rviz -> topic : /camera/pcl_filtered
	fixed frame: world_frame
	orbit: fixed frame (on the right)
```
