# AMP's Autonomous Software Stack

Autonomous Motorsports Purdue's software stack for the go-kart which will race
in the [Autonomous Karting Series](https://autonomouskartingseries.com/).
It is built on top of [ROS2 Humble](https://docs.ros.org/en/humble/index.html) and Ubuntu 22.04.

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

Now build the repo 

```
colcon build
```

And let ros2 know where the find the built packages

```
source install/local_setup.bash
```

If this works not, it should report no failed packages.

At this point, launch the simulation for a test! `ros2 launch amp_simulate robot_launch.py`. If you do not already have the right version of webots (2023b) installed, the `webots-ros2` package will prompt you to install Webots.

### Development Tips

> Note: `--symlink-install` is causing issues with Humble package search. Use `colcon build` after *all* file modifications for now.

Run `colcon build --symlink-install` after file changes to 'recompile' the project. You do not need to run this after changing python files since `--symlink-install` keep track of all new changes.

### Common Troubleshooting

If you see this: ```/usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.``` 

It is not an error and you can safely ignore it. However, if the warning bothers you, [read more](https://robotics.stackexchange.com/questions/24230/setuptoolsdeprecationwarning-in-ros2-humble/24349#24349)