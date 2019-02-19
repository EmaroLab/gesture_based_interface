# Install git
sudo apt install git
# Create workspace and clone whole project
mkdir ~/sofar_ws
cd ~/sofar_ws
git clone https://github.com/EmaroLab/gesture_based_interface.git src
cd src
git submodule init
git submodule update
# Install required stuff
. prerequisites.sh

cd ~/sofar_ws
# Issue first clean build - It will probably fail at some point 
# due to not yet accepted android sdk licenses
. src/clean_build.sh
# Accept all sdk licenses
yes | sdkmanager --licenses
# Complete first build
. src/build.sh
# Link baxter environment loader
ln -s src/baxter/baxter.sh ./baxter.sh
ln -s src/build.sh ./build.sh
ln -s src/clean_build.sh ./clean_build.sh
