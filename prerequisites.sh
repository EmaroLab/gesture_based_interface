#//bin/bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt update
sudo apt -y upgrade

sudo apt install -y ros-melodic-desktop-full freenect freeglut3* git-core cmake pkg-config build-essential libxmu-dev libxi-dev libudev* g++ python openjdk-11-jdk graphviz doxygen ros-melodic-rgbd-launch ros-melodic-openni-* ros-melodic-pcl-* ros-melodic-perception ros-melodic-perception-pcl ros-melodic-tf ros-melodic-roslib ros-melodic-orocos-kdl python-rosinstall python-rosinstall-generator python-wstool build-essential tlp ros-melodic-effort-controllers qt4-default python-scipy openjdk-11-jre openjdk-11-jdk gazebo9 ros-melodic-shape-msgs ros-melodic-pluginlib ros-melodic-class-loader ros-melodic-cv-bridge ros-melodic-cmake-modules ros-melodic-eigen-conversions ros-melodic-roslint python-pip python-imaging-tk
sudo pip install graphviz scipy tensorflow keras mttkinter Pillow

cd
rm -f sdk-tools-linux-4333796.zip
wget https://dl.google.com/android/repository/sdk-tools-linux-4333796.zip
rm -rf android-sdk
mkdir android-sdk
unzip sdk-tools-linux-4333796.zip -d android-sdk
rm sdk-tools-linux-4333796.zip

sudo rosdep init
rosdep update

sudo update-java-alternatives --set java-11-openjdk-amd64

echo '
source /opt/ros/melodic/setup.bash
source ~/sofar_ws/devel/setup.bash
export ROS_LANG_DISABLE=genlisp:gennodejs:geneus
export ANDROID_HOME=$HOME/android-sdk
export LC_NUMERIC="en_US.UTF-8"
export PATH=$PATH:$ANDROID_HOME/tools/bin
export PATH=$PATH:$ANDROID_HOME/platform-tools
export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64
export JAVA_OPTS="-XX:+IgnoreUnrecognizedVMOptions --add-modules java.se.ee"
export ROS_IP="$(hostname -I | tr -d " ")"
export ROS_HOSTNAME="$(hostname -I | tr -d " ")"
' > ~/sofar_ws/.gbi_env

echo '
function bashrc(){
    source ~/.bashrc
}

source ~/sofar_ws/.gbi_env
' >> ~/.bashrc

source ~/sofar_ws/.gbi_env

yes | sdkmanager --licenses
 
sudo adduser $USER plugdev
sudo usermod -a -G video $(whoami)

echo 2 | sudo tee -a /sys/module/usbcore/parameters/autosuspend

echo '
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
SYSFS{idVendor}=="045e", SYSFS{idProduct}=="02ae", MODE="0660", GROUP="video"
SYSFS{idVendor}=="045e", SYSFS{idProduct}=="02ad", MODE="0660", GROUP="video"
SYSFS{idVendor}=="045e", SYSFS{idProduct}=="02b0", MODE="0660", GROUP="video"
' | sudo tee /etc/udev/rules.d/66-kinect.rules

echo '
SUBSYSTEM=="usb", ATTR{idProduct}=="0200", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="0300", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="0401", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="0500", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="0600", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="0601", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="0609", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="1250", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="1260", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="1270", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="1280", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="1290", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
SUBSYSTEM=="usb", ATTR{idProduct}=="f9db", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"
' | sudo tee /etc/udev/rules.d/557-primesense-usb.rules

sudo udevadm trigger

echo 'USB_BLACKLIST="045e:02b0 045e:02ad 045e:02ae"' | sudo tee -a /etc/default/tlp
sudo systemctl restart tlp

rm -rf ~/.bgi_dep_src
mkdir ~/.bgi_dep_src
cd ~/.bgi_dep_src
git clone https://github.com/libusb/libusb
git clone https://github.com/OpenKinect/libfreenect
git clone https://github.com/EmaroLab/OpenNI
git clone -b unstable https://github.com/PrimeSense/Sensor
git clone https://github.com/arnaud-ramey/NITE-Bin-Dev-Linux-v1.5.2.23

cd ~/.bgi_dep_src/libusb
./autogen.sh
make
sudo make install

cd ~/.bgi_dep_src/libfreenect
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib64/

cd ~/.bgi_dep_src/OpenNI/Platform/Linux/CreateRedist
./RedistMaker
cd ../Redist/OpenNI-Bin-Dev-Linux-*
sudo ./install.sh

cd ~/.bgi_dep_src/Sensor/Platform/Linux/CreateRedist
./RedistMaker
cd ../Redist/Sensor-Bin-Linux-*
sudo ./install.sh 


sudo rm -f /usr/lib/libXnVNite.so
cd ~/.bgi_dep_src/NITE-Bin-Dev-Linux-v1.5.2.23/x64
sudo ./install.sh
