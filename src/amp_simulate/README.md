# Simulator

This is the webots simulator for the kart. It is not supposed to be a perfect reproduction of real life, just a testbed for our navigation code. 

After building run it using `ros2 launch amp_simulate robot_launch.py`

## Files

You will find a `kart_driver.py` file which takes input from an `AckermannDrive` msg, at topic `/cmd_ackermann`, extracts the speed and steering angle, and sets them into the simulated webots car. In other words, this files interface between ROS and webots.

