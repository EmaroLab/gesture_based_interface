#!/usr/bin/env bash
cd ~/sofar_ws
catkin_make | tee last_build.log
source devel/setup.bash
