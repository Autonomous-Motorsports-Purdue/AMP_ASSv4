# AMP's Autonomous Software Stack

Autonomous Motorsports Purdue's software stack for the go-kart which will race
in the [Autonomous Karting Series](https://autonomouskartingseries.com/).
It is built on top of [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html) as
it is the latest, and last, ROS2 distro that can run natively on the team's
NVIDIA Xavier NX computer running Ubuntu 20.04, which is limited by the distro
packaged with Jetpack: [JetPack SDK 5.0.2](https://developer.nvidia.com/embedded/jetpack-sdk-502).

**NOTE:** This repository is a ROS workspace.

## Development Setup

Make sure [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html) is
installed in your system and you have the proper environment setup.

Install webots `sudo apt install ros-foxy-webots-ros2`. Next time you launch a webots related package, such as `ros2 launch amp_simulate robot_launch.py`, it will prompt you to install the right version of Webots for your ROS distro.

### Downloading and Setting up the Repo

Clone the repo (change the link to an SSH link if you have SSH keys setup):

```bash
git clone https://github.com/Autonomous-Motorsports-Purdue/AMP_ASSv4.git
cd AMP_ASSv4
```

### Installing Dependencies and Building

```
rosdep update
rosdep install --from-paths src -iry
colcon build --symlink-install
```
