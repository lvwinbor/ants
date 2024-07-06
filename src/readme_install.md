# 1. install livox SDK and livox SDK2.
# 2. compile some packages firstly,include follow packages:
## 2.1. catkin_make -DCATKIN_WHITELIST_PACKAGES="common_private_msgs"
## 2.2. catkin_make -DCATKIN_WHITELIST_PACKAGES="plan_env"
## 2.3. catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_ros_driver"
## 2.4. cd src/bag_location/livox_ros_driver2/  && ./build.sh ROS1
## 2.5. catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_lio"
## 2.6. catkin_make -DCATKIN_WHITELIST_PACKAGES=""
