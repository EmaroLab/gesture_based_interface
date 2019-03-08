#!/usr/bin/env bash
cd ~/sofar_ws
mkdir -p apk_release
catkin_make | tee last_build.log
source devel/setup.bash
cp -f src/beacons_proximity/android_wear_pub/build/outputs/apk/release/android_wear_pub-release-unsigned.apk apk_release/beacons_proximity.apk
cp -f src/imu_wear/android_wear_pub/build/outputs/apk/release/android_wear_pub-release-unsigned.apk apk_release/imu_wear.apk
