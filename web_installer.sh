# Install git
sudo apt install -y git
# Create workspace and clone whole project
rm -rf ~/sofar_ws
mkdir ~/sofar_ws
cd ~/sofar_ws
git clone https://github.com/EmaroLab/gesture_based_interface.git src
cd src
git submodule init
git submodule update
# Install required stuff
. prerequisites.sh

cd ~/sofar_ws
source .gbi_env
# Issue first clean build - It will probably fail at some point
# due to not yet accepted android sdk licenses
. src/clean_build.sh
# Link baxter environment loader
ln -s src/_dependencies/baxter/baxter/baxter.sh
ln -s src/build.sh
ln -s src/clean_build.sh
