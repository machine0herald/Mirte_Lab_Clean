Mirte Lab Cleaner Documentation
================================

This is the top-level documentation for the **mirte_lc** ROS 2 project.
The robot autonomously navigates a lab environment, detects objects,
and cleans up using a behaviour tree architecture.

.. toctree::
   :maxdepth: 1
   :caption: Packages

Package Documentation
---------------------

Each ROS 2 package has its own API documentation:

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Package
     - Description
   * - `mirte_lc_gazebo <mirte_lc_gazebo/index.html>`_
     - Gazebo simulation worlds and launch files
   * - `mirte_lc_labclean <mirte_lc_labclean/index.html>`_
     - Main behaviour tree and lab cleaning logic
   * - `mirte_lc_moveit_cpp <mirte_lc_moveit_cpp/index.html>`_
     - MoveIt C++ action server for arm control
   * - `mirte_lc_msgs <mirte_lc_msgs/index.html>`_
     - Custom ROS 2 message, service, and action definitions
   * - `mirte_lc_nav2 <mirte_lc_nav2/index.html>`_
     - Nav2 coverage navigation and frontier-based exploration
   * - `mirte_lc_vision <mirte_lc_vision/index.html>`_
     - Object detection and 3D localisation via YOLO + point cloud