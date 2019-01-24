# Bioloid_pose_estimation

This project was done for the coursework of Robotics Engineering master by Andrea Romdhana and Francesco Laneve.
The aim of this work is to determine the pose of a humanoid robot in an objects-free environment.

Using a low budget setup, composed by a depth camera (Kinect) and an IMU sensor, we propose an approach to estimate the 
position of a humanoid robot's center of mass and the Yaw-Pitch-Roll angles of the robot's trunk, which is useful to locate it 
and to apply movement policies, allowing the robot to be autonomous.

# Requirements:
* Linux Ubuntu 16.04
* Ros Kinetic
* Kinect One or equivalent
* Imu Sensor (phidgets 3/3/3)
* A configured Bioloid (see below)

# Setup:

It is requested to install [freenect2](https://github.com/OpenKinect/libfreenect2).

In order to use Rosnode in combination with the Bioloid Robot see [RaspBioloid](https://github.com/roncapat/RaspBioloid), this 
package is necessary for the IMU management.

# Launching the nodes:
You can start the all the nodes by launching:
```
roslaunch bioloid_pose_estimation bioloid_setup.launch
```
