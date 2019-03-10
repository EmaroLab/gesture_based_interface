#!/usr/bin/env bash
cd ~/sofar_ws
rm -rf build devel apk_release
mkdir apk_release
find . -name "*.apk" -exec rm -f {} \;
catkin_make -DCATKIN_WHITELIST_PACKAGES="rosjava_messages;genjava;rosjava_build_tools"  | tee last_build.log
catkin_make -DCATKIN_WHITELIST_PACKAGES="imu_wear;beacons_proximity_sub" -j1 | tee -a last_build.log
yes | sdkmanager --licenses
catkin_make -DCATKIN_WHITELIST_PACKAGES=""  | tee -a last_build.log
exec bash
cp -f src/beacons_proximity/android_wear_pub/build/outputs/apk/release/android_wear_pub-release-unsigned.apk apk_release/beacons_proximity.apk
cp -f src/imu_wear/android_wear_pub/build/outputs/apk/release/android_wear_pub-release-unsigned.apk apk_release/imu_wear.apk
