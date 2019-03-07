#!/usr/bin/env bash
cd ~/sofar_ws
catkin_make | tee last_build.log
source devel/setup.bash
cp -f src/imu_wear/android_wear_pub/build/outputs/apk/debug/android_wear_pub-debug.apk imu_wear-debug.apk
cp -f src/beacons_proximity/android_wear_pub/build/outputs/apk/debug/android_wear_pub-debug.apk beacons_proximity-debug.apk
