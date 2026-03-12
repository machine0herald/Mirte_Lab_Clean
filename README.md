# Mirte Master Lab Cleanup Robot
![Status](https://img.shields.io/badge/status-in%20progress-yellow)

## Installation

1. Clone repository to the src folder of your ros2 workspace
    - For developers
    ```sh
    cd ros2_ws/src 
    git clone https://github.com/machine0herald/Mirte_Lab_Clean
    ```
    - For contributors
    ```sh
    cd ros2_ws/src 
    git remote add origin https://github.com/machine0herald/Mirte_Lab_Clean
    git pull origin main
    ```

2. Update submodules recursively
    ```sh
    cd ..
    git submodule update --init --recursive
    ```

3. install dependencies
    ```sh
    vcs import src/ < src/mirte_lc/sources.repos
    ```

4. update mirte_ros_packages submodule
    ```sh
    cd src/mirte-ros-packages && git submodule update --init --recursive
    ```

5. Install mirte ros packages' rosdeps and build
    ```sh
    rosdep install -y --from-paths src/ --ignore-src --rosdistro humble
    colcon build --symlink-install
    ```
## Demo

[![Watch the video](thumbnail.png)](Test_node_sim.mp4)