# Simulator

This is the webots simulator for the kart. It is not supposed to be a perfect reproduction of real life, just a testbed for our navigation code. 

After building run it using `ros2 launch amp_sim robot_launch.py`

## Files

You will find a `kart_driver.py` file which takes input from an `AckermannDrive` msg, at topic `/cmd_ackermann`, extracts the speed and steering angle, and sets them into the simulated webots car. In other words, this file interfaces between ROS and webots.

`robot_launch.py` launches the webots instance, as well as the `webots_driver` package, which does the heavy lifting so that `kart_driver.py` can control the simulation via a nice Python API. Additionally, it launches the lane detection code in the `amp_lane` package.

To change the simulation world, go into `robot_launch.py` and change the `world` launch argument to some files ending in `.wbts` in the `worlds/` directory.

Through the magic of `webots_driver.py`, the webots simulation also publishes a topic `vehicle/camera` of type `Image` that can be used for navigation purposes.