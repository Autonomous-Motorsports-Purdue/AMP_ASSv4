# AMP's Autonomous Software Stack

Autonomous Motorsports Purdue's software stack for the go-kart which will race
in the [Autonomous Karting Series](https://autonomouskartingseries.com/).
It is built on top of [ROS2 Hunble](https://docs.ros.org/en/humble/index.html) and Ubuntu 22.04.

**NOTE:** This repository is a ROS workspace.

## Development Setup

Make sure [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) is
installed in your system and you have the proper environment setup.

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

Next time you launch a webots related package, such as `ros2 launch amp_simulate robot_launch.py`, the `webots-ros2` package` will prompt you to install the right version of Webots for your ROS distro.

### Common Troubleshooting

Seeing this: ```/usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.``` 

This is not an error and you can safely ignore it. However, if the warning bothers you, [Read more here](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/)