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

At this point, launch the simulation for a test! `ros2 launch amp_sim robot_launch.py`. If you do not already have the right version of webots (2023b) installed, the `webots-ros2` package will prompt you to install Webots.

### Development Tips

> Note: `--symlink-install` is causing issues with Humble package search. use `colcon build` for now

Run `colcon build --symlink-install` after file changes to 'recompile' the project. The one exception is python files, which does not require a colcon invocation due to the usage of `--symlink-install`, which automatically reflects changes in the python source code to the runnable versions.  

### Common Troubleshooting

Seeing this: ```/usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.``` 

This is not an error and you can safely ignore it. However, if the warning bothers you, [Read more here](https://robotics.stackexchange.com/questions/24230/setuptoolsdeprecationwarning-in-ros2-humble/24349#24349)