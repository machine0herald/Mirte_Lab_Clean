apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit-servo \
    ros-humble-moveit-visual-tools \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-slam-toolbox \

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

git submodule update --init --recursive

vcs import src/ < mirte_lc/sources.repos
vcs import src/ < src/mirte-gazebo/sources.repos
rosdep install -y --from-paths src/ --ignore-src --rosdistro humble

cd mirte-ros-packages && git submodule update --init --recursive && cd ..

apt-get update
rosdep install --from-paths src --ignore-src -r -y

colcon build
source install/setup.bash
