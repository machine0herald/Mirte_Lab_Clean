# Mirte Master Lab Cleanup Robot
![Status](https://img.shields.io/badge/status-in%20progress-yellow)

## Desktop Installation

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
    cd src/mirte-ros-packages && git submodule update --init --recursive && cd ../..
    ```

5. Install mirte ros packages' rosdeps and build
    ```sh
    rosdep install -y --from-paths src/ --ignore-src --rosdistro humble
    colcon build --symlink-install
    ```

## Robot Installation
The real robot does not need the gazebo packages,so to save on storage, 
we exclude it manually/.

1. Create the folder and add the repo
    ```sh
    cd ~/mirte_ros_ws/src && mkdir mirte_lc && cd mirte_lc
    git init
    git remote add origin https://github.com/machine0herald/Mirte_Lab_Clean
    ```

2. Init sparse checkout.
   ```sh
   git sparse-checkout init --no-cone
   ```

3. Edit .git/info/sparse-checkout with the following changes:
   ```lua
    /*
    !src/gazebo_pkg/
   ```

4. Apply.
    ```sh
    git read-tree -mu HEAD
    ```

5. Pull changes.
   ```sh
   git pull origin main
   ```