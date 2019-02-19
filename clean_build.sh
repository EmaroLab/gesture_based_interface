#!/usr/bin/env bash
cd ~/sofar_ws
rm -rf build devel
catkin_make -DCATKIN_WHITELIST_PACKAGES="rosjava_messages;genjava;rosjava_build_tools"  | tee last_build.log
catkin_make -DCATKIN_WHITELIST_PACKAGES=""  | tee -a last_build.log
source devel/setup.bash
