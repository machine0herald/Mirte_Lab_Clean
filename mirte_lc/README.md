# Metapackage

Metapackage for mirte_lc ROS 2 packages containing all rosdeps.

## Contents

- Package: mirte_lc
- Path: `mirte_lc/mirte_lc`

## Quickstart

1. Source your workspace:

    ```bash
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ```

2. install rosdeps defined in the package

    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
