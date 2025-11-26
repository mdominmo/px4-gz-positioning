# PX4 SITL Gazebo Sim Local Positioning
![GitHub tag](https://img.shields.io/github/v/tag/mdominmo/px4-gz-positioning)
![GitHub](https://img.shields.io/github/license/mdominmo/px4-gz-positioning)

This repository aims to provide local positioning to simulated **PX4-based** UAVs from the **Gazebo Transport API**.

According to [PX4 docs](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry), it is posible to make the UAVs work with external local positioning in order to implement or integrate with visual-based navigation techniques. This feature is taken here to attach the positioning given by the **Gazebo** [Pose Publisher](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1PosePublisher.html) contained in the UAV **sdf** models.

Since the position given by the pose publisher is referenced to the Gazebo simulation world's origin, it provides a common local positioning between UAVs and the rest of the simulation environment.

This feature, allows to work with **Software-In-The-Loop** simulations without **GNSS transformations**, avoiding to deal with their respective precision errors. 

## Dependencies
- Gazebo Harmonic
- ros2 humble
- PX4 Autopilot (v1.15.0) and Gazebo simulation models. 
- px4_msgs

## Getting Started

### Prerequisites

Clone or fork this repo inside your ros2 workspace.

```
cd ros2_ws/
git fork https://github.com/mdominmo/px4-gz-positioning.git
```

If you already had the previously mentioned [dependencies](#dependencies), then you can go to the [Installation](#installation) section.

1) Install ROS2 Humble following the [official documentation]([ros2](https://docs.ros.org/en/humble/Installation.html)).
2) Download the [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) repository to get the UAVs sdf models, or follow the complete [installation steps](https://docs.px4.io/main/en/simulation/) to get a complete PX4-SITL environment.
    
    ```
    git clone https://github.com/PX4/PX4-Autopilot.git
    ```
3) Install Gazebo Harmonic and Gazebo Transport API. Follow this step in case that you were planning to work in a device apart from the Gazebo server and the PX4-Autopilot SITL suite (Remember that if you have follow the [PX4-Autopilot installation](https://docs.px4.io/main/en/simulation/) from the previous step, these dependencies should be present so you can go to **4)**:

    ```
    apt-get update && \
    apt-get install -y \
    ros-humble-ros-gzharmonic \
    gz-harmonic
    ```
4) Download the [px4_msgs]([px4_msgs](https://github.com/PX4/px4_msgs)) to your ros2 workspace and build it:

    ```
    git clone https://github.com/PX4/px4_msgs.git && \
    cd px4_msgs && \
    git checkout release/1.15
    ```


### Installation

Inside the ros2_ws:

```
colcon build && source install/setup.bash
```

## Usage

### PX4 Parameters configuration
Since this tool uses the PX4 visual navigation feature, a simulated **VIO-based** like the **x500_vision** multirrotor model is needed when running the px4 sitl instance initialization script. 

```
export PX4_SIM_MODEL=x500_vision PX4_SYS_AUTOSTART=4005 ${PX4_FOLDER}/build/px4_sitl_default/bin/px4
``` 

Neither **x500**, nor **x500_mono_cam** can be used for this purpose because of the lack of an **odometry publisher**.

In order to make the autopilot working without GPS sensor fusioning you need to set these parameters to the following values:

[!Note] You can do this via [QGround Control](https://qgroundcontrol.com/).

| Parameter | Value |
|--------------|--------------|
| PX4_PARAM_EKF2_EV_CTRL  | 11  |
| PX4_PARAM_EKF2_HGT_REF  | 3  |
| PX4_PARAM_EKF2_GPS_CTRL  | 0  |
| PX4_PARAM_EKF2_BARO_CTRL  | 0  |
| PX4_PARAM_EKF2_RNG_CTRL  | 0  |


This tool will inject local positioning by publishing in the `/px4_<n>/fmu/in/vehicle_visual_odometry` ros2 topic where **n** is 1 by default. Remember to export the **PX4_UXRCE_DDS_NS** parameter with a subsecuent value when running a SITL simulation with one or multiple PX4 instances.

Once the Autopilots are configured it is posible to run multiple positioning nodes by executing:

```
ros2 launch px4_gz_positioning launch.py num_vehicles:=n
```

Where **n** is the number of vehicles in wich the vehicle visual odometry will be injected. This will create a set of **n** ros2 nodes publishing from vehicles **px4_1** to **px4_n**. 

[!Note] By the moment, you will need to change to **OFFBOARD** mode manually because of the safety flightchecks.

## Support

To request technical support, please open an issue explaining the case.

## Acknowledgments

- [Mocap-PX4 Bridge](https://github.com/SaxionMechatronics/mocap_px4_bridge)
- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- [Gazebo](https://gazebosim.org/home)

