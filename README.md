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
// launch nodes used by Kinect
roslaunch openni_launch openni.launch
// configuration: generation of environments according to the orientation angle of the Kinect
roslaunch pose_estimation config.launch //launcher per generare environment per ogni angolazione
// estimation of the position of human's center of mass
roslaunch pose_estimation pose_estimator.launch
// service for setting the orientation angle of the Kinect 
// (node inside pose estimator that publishes the angle on the topic tilt_angle)
rosservice call /move_kinect "angle: <float>"
```

//rviz -> topic : /camera/pcl_filtered
	fixed frame: camera_link
	orbit: fixed frame (destra) 
//servizi per 
rostopic pub //service passi angolo per telecamera -> nodo nel pose estimator
//pubblica angolo su tilt angle
