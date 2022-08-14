rosdep update
sudo apt update
sudo apt dist-upgrade

sudo apt install ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon

sudo apt install python3-wstool

mkdir -p ~/robotic_task/

cd  ~/robotic_task/

git clone https://github.com/utkusaglm/robotic_arm.git

cd ~/robotic_task/robotic_arm/src

rosdep install -y --from-paths . --ignore-src --rosdistro noetic

sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt update

cd ~/robotic_task/robotic_arm/

catkin init
catkin build

source ~/robotic_task/robotic_arm/devel/setup.bash

echo 'source ~/robotic_task/robotic_arm/devel/setup.bash' >> ~/.bashrc
