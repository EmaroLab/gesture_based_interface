#!/usr/bin/env bash
cd ~/sofar_ws
rm -rf build devel
catkin_make -DCATKIN_WHITELIST_PACKAGES="rosjava_messages;genjava;rosjava_build_tools"  | tee last_build.log
catkin_make -DCATKIN_WHITELIST_PACKAGES="imu_wear;beacons_proximity_sub" -j1 | tee -a last_build.log
yes | sdkmanager --licenses
catkin_make -DCATKIN_WHITELIST_PACKAGES=""  | tee -a last_build.log
source devel/setup.bash
cp -f src/imu_wear/android_wear_pub/build/apk/debug/android_wear_pub-debug.apk imu_wear-debug.apk
cp -f src/beacons_proximity/android_wear_pub/build/apk/debug/android_wear_pub-debug.apk beacons_proximity-debug.apk
