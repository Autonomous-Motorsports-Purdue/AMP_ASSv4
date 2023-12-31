# AMP's Autonomous Software Stack

Autonomous Motorsports Purdue's software stack for the go-kart which will race
in the [Autonomous Karting Series](https://autonomouskartingseries.com/).
It is built on top of [ROS2 Humble](https://docs.ros.org/en/humble/index.html) and Ubuntu 22.04.

**NOTE:** This repository is a ROS workspace.

## Development Setup

Make sure [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) is
installed in your system and you have the proper environment setup.

If you are deploying to the Jetson, please read [Jetson Setup Guide](JETSON_SETUP.md) for how to wire and run code on the Jetson

### Downloading and Setting up the Repo

Set up your Git SSH keys:

[Step 1](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
[Step 2](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
[Check](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/testing-your-ssh-connection)

Clone the repo:

```bash
git clone git@github.com:Autonomous-Motorsports-Purdue/AMP_ASSv4.git
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

And let ROS2 know where the find the built packages

```
source install/local_setup.bash
```

If this works out, it should report no failed packages.

At this point, launch the simulation for a test! `ros2 launch amp_simulator robot_launch.py`. If you do not already have the right version of webots (2023b) installed, the `webots-ros2` package will prompt you to install Webots.

### Development Tips

> Note: `--symlink-install` is causing issues with Humble package search. Use `colcon build` after *all* file modifications for now.

Run `colcon build --symlink-install` after file changes to 'recompile' the project. You do not need to run this after changing python files since `--symlink-install` keep track of all new changes.

### Common Troubleshooting

If you see this: ```/usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.``` 

It is not an error and you can safely ignore it. However, if the warning bothers you, [read more](https://robotics.stackexchange.com/questions/24230/setuptoolsdeprecationwarning-in-ros2-humble/24349#24349)