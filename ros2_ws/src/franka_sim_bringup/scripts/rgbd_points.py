#!/usr/bin/env python3
"""Publish /zed_x_mini/rgbd/points from the simulated depth and colour images.

Gazebo Harmonic's own RGB-D point cloud is labelled with the optical frame but
filled in the sensor frame (+x forward), and is slightly distorted (about 2.6 deg
on the table). As a ROS depth pipeline does, this node instead back-projects each
depth pixel with CameraInfo, so the organised cloud is in left_camera_link_optical,
with NaN where the depth is invalid.
"""
import array

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

FIELDS = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
          for i, name in enumerate(('x', 'y', 'z', 'rgb'))]


def stamp(msg):
    return msg.header.stamp.sec, msg.header.stamp.nanosec


class RgbdPoints(Node):
    def __init__(self):
        super().__init__('zed_x_mini_points')
        self.rays = None
        self.cloud = None
        self.pending = {}
        self.points = self.create_publisher(PointCloud2, '/zed_x_mini/rgbd/points', 2)
        self.create_subscription(CameraInfo, '/zed_x_mini/rgbd/camera_info', self.on_info, 2)
        self.create_subscription(Image, '/zed_x_mini/rgbd/depth_image',
                                 lambda msg: self.on_image('depth', msg), 2)
        self.create_subscription(Image, '/zed_x_mini/rgbd/image',
                                 lambda msg: self.on_image('color', msg), 2)

    def on_info(self, info):
        fx, _, cx, _, fy, cy = info.k[:6]
        self.rays = ((np.arange(info.width, dtype=np.float32) - cx) / fx,
                     (np.arange(info.height, dtype=np.float32)[:, None] - cy) / fy)

    def on_image(self, kind, msg):
        # Depth and colour of one frame carry the same stamp; keep only the newest frame.
        self.pending = {k: m for k, m in self.pending.items() if stamp(m) == stamp(msg)}
        self.pending[kind] = msg
        if len(self.pending) == 2 and self.rays is not None:
            self.publish(self.pending.pop('depth'), self.pending.pop('color'))

    def publish(self, depth, color):
        if depth.encoding != '32FC1' or color.encoding != 'rgb8':
            self.get_logger().error(f'Unexpected encodings {depth.encoding}, {color.encoding}',
                                    throttle_duration_sec=10.)
            return
        h, w = depth.height, depth.width
        if self.cloud is None or self.cloud.shape[:2] != (h, w):
            # Reused for every frame: publish() copies the message before returning.
            self.data = array.array('B', bytes(16 * h * w))
            self.cloud = np.frombuffer(self.data, np.float32).reshape(h, w, 4)
        z = np.frombuffer(depth.data, np.float32).reshape(h, depth.step // 4)[:, :w]
        np.multiply(self.rays[0], z, out=self.cloud[..., 0])
        np.multiply(self.rays[1], z, out=self.cloud[..., 1])
        self.cloud[..., 2] = z
        self.cloud[~np.isfinite(z) | (z <= 0), :3] = np.nan
        # rgb is the float whose little-endian bytes are B, G, R, 0.
        rgb = np.frombuffer(color.data, np.uint8).reshape(h, color.step)[:, :3 * w]
        self.cloud.view(np.uint8)[..., 12:15] = rgb.reshape(h, w, 3)[..., ::-1]
        self.points.publish(PointCloud2(
            header=depth.header, height=h, width=w, fields=FIELDS, is_bigendian=False,
            point_step=16, row_step=16 * w, is_dense=False, data=self.data))


def main():
    rclpy.init()
    node = RgbdPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
