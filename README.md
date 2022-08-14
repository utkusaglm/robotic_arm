# robotic_arm
* Please install the moveit noetic : https://wiki.ros.org/noetic/Installation/Ubuntu
## There are two ways to run the code;
  - You can use run_build.sh script to built packages for you.
  OR
  - You can manualy built with following the run_build.sh
  
  ```
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
  ```
- Then please move to the;
```cd ~/robotic_task/robotic_arm/``` and run ```roslaunch panda_moveit_config demo.launch``` on the terminal.
- In another terminal please run the ```rosrun robotic robotic.py```
- Video : https://www.youtube.com/watch?v=SjgvFNFlToI
![Ekran Resmi 2022-08-15 00 13 33](https://user-images.githubusercontent.com/58150504/184555134-fe33974c-85d8-4c15-9720-fb7239c884a8.png)
