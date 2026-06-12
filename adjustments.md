# MIRTE Lab Clean — Setup & Adjustments

This document covers required modifications to upstream packages and initial robot setup procedures.

---

## Dependencies

```bash
sudo apt install ros-humble-image-pipeline
```

---

## Clock Synchronisation (manual fallback)

If Chrony is not available, clocks can be synchronised manually:

```bash
sudo date -s "YYYY-MM-DD HH:MM:SS"
```

Example:

```bash
sudo date -s "2026-01-01 12:01:01"
```

> Chrony is preferred for persistent synchronisation. See the [quickstart](quickstart.md) for the full Chrony setup.

---

## Upstream Package Modifications

The following changes must be applied to upstream packages before building.

### 1. `mirte-gazebo` — Gazebo launch file

**File:** `mirte-gazebo/launch/gazebo_mirte_world_generated.launch.xml`

Add the `mirte_lc_gazebo` model path and enable verbose output:

```xml
<launch>
  <set_env name="GAZEBO_MODEL_PATH"
    value="$(env GAZEBO_MODEL_PATH ''):$(find-pkg-share mirte_gazebo)/models"/>
  <set_env name="GAZEBO_MEDIA_PATH"
    value="$(env GAZEBO_MEDIA_PATH ''):$(find-pkg-share mirte_gazebo)/media"/>

  <!-- Add mirte_lc_gazebo worlds to model path -->
  <set_env name="GAZEBO_MODEL_PATH"
    value="$(env GAZEBO_MODEL_PATH ''):$(find-pkg-share mirte_lc_gazebo)/worlds"/>

  <arg name="gui" default="true"/>
  <arg name="generated_world"
    default="$(find-pkg-share mirte_gazebo)/worlds/robocupjunior_simple.world"/>

  <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py">
    <arg name="world" value="$(var generated_world)"/>
    <arg name="gui"   value="$(var gui)"/>
    <!-- Enable verbose Gazebo output -->
    <arg name="verbose" value="true"/>
  </include>
</launch>
```

---

### 2. `mirte_description` — Ultrasonic sensor URDF

**File:** `mirte-ros-packages/mirte_description/urdf/ultrasonic.urdf`

Gazebo publishes invalid ranges when the measured distance exceeds `max`. Increase the max range to avoid this:

```xml
<range>
  <min>0.03</min>
  <!-- Increased from default to prevent invalid range values -->
  <max>100</max>
  <resolution>0.01</resolution>
</range>
```

---

### 3. `mirte_description` — Arm URDF mimic joints

**File:** `mirte-ros-packages/mirte_description/urdf/arm.urdf.xacro`

Rename mimic joints to avoid naming conflicts. Append `_mimic` to the following joint names:

| Original name | Renamed to |
|---|---|
| `_Gripper_joint_r` | `_Gripper_joint_r_mimic` |
| `gripper_link_joint_l` | `gripper_link_joint_l_mimic` |
| `_gripper_link_joint_r` | `_gripper_link_joint_r_mimic` |

---

### 4. `mirte_moveit_config` — Arm group states

**File:** `mirte-ros-packages/mirte_moveit_config/config/mirte_master.srdf`

Add the following group states under the `mirte_arm` move group (after line 29, below the `home` state):

```xml
<group_state name="place_right" group="mirte_arm">
    <joint name="shoulder_pan_joint"  value="0.3"/>
    <joint name="shoulder_lift_joint" value="-0.6"/>
    <joint name="elbow_joint"         value="1.5"/>
    <joint name="wrist_joint"         value="0.6"/>
</group_state>

<group_state name="place_left" group="mirte_arm">
    <joint name="shoulder_pan_joint"  value="-0.3"/>
    <joint name="shoulder_lift_joint" value="-0.6"/>
    <joint name="elbow_joint"         value="1.5"/>
    <joint name="wrist_joint"         value="0.6"/>
</group_state>

<group_state name="standby" group="mirte_arm">
    <joint name="shoulder_pan_joint"  value="0.0"/>
    <joint name="shoulder_lift_joint" value="-0.5"/>
    <joint name="elbow_joint"         value="-1.3"/>
    <joint name="wrist_joint"         value="-1.3"/>
</group_state>

<group_state name="vigilant" group="mirte_arm">
    <joint name="shoulder_pan_joint"  value="0.0"/>
    <joint name="shoulder_lift_joint" value="-0.5"/>
    <joint name="elbow_joint"         value="-1.3"/>
    <joint name="wrist_joint"         value="-0.4"/>
</group_state>
```

Named pose reference:

| Pose | Description |
|---|---|
| `vigilant` | Upright standby, arm raised — startup default |
| `standby` | Deployed forward, ready for object approach |
| `place_left` | Drop position for target objects |
| `place_right` | Drop position for trash objects |

---

### 5. `mirte_description` — Gripper camera (Gazebo)

**File:** `mirte-ros-packages/mirte_description/urdf/arm.urdf.xacro`

Add the gripper camera optical frame, Gazebo sensor plugin, and grasp fix plugin:

```xml
<!-- Optical frame for gripper camera -->
<joint name="gripper_camera_rgb_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="1.5707963267949 -1.5707963267949 0"/>
    <parent link="gripper_camera_rgb_frame"/>
    <child  link="gripper_camera_rgb_optical_frame"/>
</joint>
<link name="gripper_camera_rgb_optical_frame"/>

<!-- Gazebo camera sensor -->
<gazebo reference="gripper_camera_rgb_optical_frame">
    <sensor name="gripper_camera_sensor" type="camera">
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>30</update_rate>
        <camera name="gripper_camera">
            <horizontal_fov>1.047</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.02</near>
                <far>6</far>
            </clip>
            <distortion>
                <k1>0.0</k1><k2>0.0</k2><k3>0.0</k3>
                <p1>0.0</p1><p2>0.0</p2>
            </distortion>
        </camera>
        <ros>
            <namespace>gripper_camera</namespace>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        </ros>
        <plugin name="gripper_camera_controller" filename="libgazebo_ros_camera.so">
            <camera_name>gripper_camera</camera_name>
        </plugin>
    </sensor>
</gazebo>

<!-- Gazebo grasp fix plugin -->
<gazebo>
    <plugin filename="libgazebo_grasp_fix.so" name="gazebo_grasp_fix">
        <arm>
            <arm_name>finger_gripper</arm_name>
            <palm_link>wrist</palm_link>
            <gripper_link>gripper</gripper_link>
            <gripper_link>_Gripper_r</gripper_link>
            <gripper_link>_gripper_link_l</gripper_link>
            <gripper_link>gripper_finger_l</gripper_link>
            <gripper_link>_gripper_link_r</gripper_link>
            <gripper_link>gripper_finger_r</gripper_link>
        </arm>
        <forces_angle_tolerance>100</forces_angle_tolerance>
        <update_rate>4</update_rate>
        <grip_count_threshold>4</grip_count_threshold>
        <max_grip_count>8</max_grip_count>
        <release_tolerance>0.005</release_tolerance>
        <disable_collisions_on_attach>false</disable_collisions_on_attach>
    </plugin>
</gazebo>
```

---

## MIRTE Master — Initial Robot Setup

### 1. Flash and connect

1. Insert an SD card with the MIRTE image and wait for flashing to complete.
2. Power on the MIRTE Master and wait for the Wi-Fi hotspot to appear.
3. Connect your PC to the MIRTE hotspot.
4. SSH into the robot:

```bash
ssh mirte@mirte_local
# password: mirte_mirte
```

---

### 2. Configure motor pins (if needed)

If the motors are wired differently, adjust the pin mapping:

```
~/mirte_ws/src/mirte-ros-packages/mirte_bringup/telemetrix_config/mirte_master_config.yaml
```

---

### 3. Tune PID values

Edit the base controller PID config on the robot:

```bash
sudo nano /opt/ros/humble/share/mirte_base_control/config/mirte_base_control.yaml
```

Set the following values for all motors:

| Parameter | Value |
|---|---|
| P | 0.5 |
| I | 2.0 |
| D | 0.0 |

---

### 4. Calibrate voltage ranges

Run the following commands on the robot:

```bash
ros2 run mirte_test mirte_master_set_voltage_ranges
ros2 run mirte_test mirte_master_calibrate
```

---

### 5. Set arm to home pose

1. Turn off ROS2 on the robot.
2. Physically reposition the arm joints to match the `home` pose defined in the SRDF.
3. Reassemble any arm components that were moved during calibration.