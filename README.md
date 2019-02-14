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

## How to run the simulator
Enter in the workspace
```
cd ~/sofar_ws
```

Execute the file passing as parameter "sim". In this way you can use the simulator in your machine.
If instead you want to work on the real robot you have to pass the serial number of the Baxter.
```
./baxter.sh sim
```

Launch all the nodes used for the simulation.
```
roslaunch baxter_gazebo baxter_world.launch
```

Launch the nodes used for managing the movements and the files.
```
rosrun BaxterGBI_pbr pbr_server_baxter.py
rosrun BaxterGBI_pbr pbr_server_filesys.py
rosrun BaxterGBI_pbr joint_recorder_node.py
rosrun BaxterGBI_pbr mirror_server.py
```


Then you can test it using the following client nodes:
```
rosrun BaxterGBI_pbr pbr_client_TEST.py mode arg1 arg2 ...

rosrun BaxterGBI_pbr mirror_client.py
```

Where, based on mode you can ask for a specific server (and you have to pass specific parameters).


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
