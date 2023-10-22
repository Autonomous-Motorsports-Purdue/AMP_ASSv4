# Jetson Setup Guide

The goal of this document is to allow anyone to physically set up the Jetson as well as deploy and run code. In the past, only a few people knew how to use the Jetson, so it was hard for Mechanical/Electrical as well as other software people to help out on race day. Additionally, people had to work around the Jetson because they didn't know how to fix common problems.

In the future, anyone should be able to test their code on the Jetson or use the Jetson for whatever purpose without needing to ask for help. However, newer members should still ask for help before handling the hardware until they get used to working with it.

If you see this document is out of date, feel free to update it.

## Physical connection Guide

How to physically connect the Jetson, the ZED camera, and the Velodyne LIDAR.

Each device needs to receive power as well as have a cable to share data with the Jetson. 

- The Jetson itself gets power through this cable. When you plug the Jetson in, there should be a green light on the board. Password is `MEsucksA$$`

- Then, you can work with the Jetson directly via a monitor + keyboard + mouse, or connect to it using a separate device via cable, wifi hotspot. 

	- For the monitor + keyboard + mouse option, just connect a monitor and mouse to the Jetson via usb, and connect a monitor via either HDMI or displayport. Then the Jetson behaves like a desktop machine.

	- For the cable option, find a usb-mini to A cable OR ethernet, and connect the Jetson to a separate SSH supported device with the cable. If the computer supports SSH, then you can log on to the Jetson with `ssh amp@192.168.55.1` and password `MEsucksA$$`. Aternatively, the Jetson hosts a VNC server with password `pl$_W0rk` and display is `:2`

	- For the hotspot option, no cable is neccessary, but the external device should be connected to the WIFI hotspot hosted by the Jetson when it is on. The SSID is `amp-xavier-nx` (IP `10.42.0.1` and password is `G0_AMP!!` with a zero. After the wifi is connected, SSH into the device using the (different!) credentials on the bullet above.

	- During race day, we may need a remote connection to the Jetson over distances too far for hotspot. Then we will create a wireless ethernet connection between two Ubiquiti Rockets, one conencted to the Jetson and the other connected to a remote machine. Both Rockets are connected to their respective machines via ethernet, and all blue bars on the Rocket should be on for a sucessful connection. Afterwards, do the same as if yo were connected via a physical ethernet cable.

- The Velodyne first connects to an adapter, which then has ports for signal (via Ethernet) and power. The ethernet cable connects to the LAN port on the Jetson. Since the Jetson has only one LAN port, in the case we need to make multiple ethernet connections, we'll use a splitter to create more ports at the cost of bandwidth.

> Note: we have set up a static IPv4 conenction on the Jetson such that the Velodyne will be automatically recognized upon connection. If not, the Velodyne's IP is either 192.168.2.201 or 192.168.2.77

- The ZED simply connects via its USB cable. Unlike the Velodyne, the ZED functions as a normal camera without needing special software to get data. You can try connecting the ZED to a computer and viewing it using the default camera application.
