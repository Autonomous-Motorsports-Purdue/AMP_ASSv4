# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 Tesla driver."""

import pkgutil
import sys
import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image, PointCloud2
# Ridiculous - A separaate package to parse PC2 points?
from sensor_msgs_py.point_cloud2 import read_points_numpy
from cv_bridge import CvBridge

from ackermann_msgs.msg import AckermannDrive
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node

from matplotlib import pyplot as plt

from amp_lane.nanosam import model

CONTROL_COEFFICIENT = 0.0015

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')

        self.bridge = CvBridge()

        # ROS interface
        self._ackermann_publisher = self.create_publisher(AckermannDrive, 'cmd_ackermann', 1)
        self._depth_publisher = self.create_publisher(Image, '/vehicle/camera/image_raw', 4)

        qos_camera_data = qos_profile_sensor_data
        qos_camera_data.reliability = QoSReliabilityPolicy.RELIABLE

        qos_pc = qos_profile_sensor_data
        qos_pc.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(Image, 'vehicle/camera/image_color', self.__on_camera_image, qos_camera_data)
        self.create_subscription(Image, 'vehicle/range_finder/image', self._on_depth, qos_camera_data)

        self.create_subscription(PointCloud2, 'resultpoints', self.dd, qos_pc)


        self.last_depth = None

        
    def dd(self, pc):
        print('got pc')

    def _on_depth(self, message):
        img = np.frombuffer(message.data, dtype=np.float32).reshape((message.height, message.width))
        self.last_depth = img

        # pc = read_points_numpy(message)
        
        # print(pc.shape, pc.dtype)

    def __on_camera_image(self, message: Image):
        img = message.data
        img = np.frombuffer(img, dtype=np.uint8).reshape((message.height, message.width, 4))
            
        # Careful, this is BGR! opencv uses BGR but pretty much anything else is in RGB  
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        mask, anns = model.find_road_mask(img)
        
        # low = -1/2 * message.height
        # high = -1/10 * message.height
        # img_road = img[int(low):int(high), :]

        # # Segment the image by color in HSV color space
        # img_hsv = cv2.cvtColor(img_road, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(img_hsv, np.array([0, 0, 200]), np.array([180, 50, 255]))

        def publish_steer(speed, error):
            command_message = AckermannDrive()
            command_message.speed = speed
            command_message.steering_angle = 0.0
                
            command_message.steering_angle = error * CONTROL_COEFFICIENT
            self._ackermann_publisher.publish(command_message)

        if mask is None:
            publish_steer(10, 0)

        if isinstance(mask, dict):
            mask = mask['segmentation']
        vis = anns
        # vis = mask.copy()
        # vis = img + np.dstack([vis, vis, vis]).astype(np.uint8)

        mask_size = mask.sum()

        if mask_size == 0:
            publish_steer(10, 0)

        # # Average x index of a mask pixel, weighted by Y
        # avg_x = (mask * yv * xv).sum() / (mask * yv).sum()
        # # Average Y index of a mask pixel
        # avg_y = (mask * yv).sum() / mask_size

        # Topmost point of mask
        nonz = np.nonzero(mask)
        y_val = int(np.percentile(nonz[0], 15, axis=0))
        top_y = y_val
        top_x = (mask[y_val] * np.arange(0, message.width, 1)).sum() / mask[y_val].sum()

        print('Center', top_x, top_y)

        cv2.circle(vis, (int(top_x), int(top_y)), 30, (255, 0, 0), -1)
        cv2.imshow("Steer direction", vis)

        publish_steer(top_x - message.width / 2, 30.0)

        if self.last_depth is not None:
            assert(self.last_depth.size == message.width * message.height)

            masked_depth = self.last_depth.copy()
            masked_depth[~mask] = 0

            print('Publishing depth')
            pub_msg = self.bridge.cv2_to_imgmsg(
                masked_depth)
            pub_msg.header = message.header
            self._depth_publisher.publish(pub_msg)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    follower = LaneFollower()
    rclpy.spin(follower)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
