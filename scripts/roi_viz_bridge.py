#!/usr/bin/env python3
# Copyright (c) 2026，D-Robotics.
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

"""Republish ROIPointClouds as a standard PointCloud2 for Foxglove / RViz.

Foxglove and RViz cannot render the custom ROIPointClouds message directly.
This optional debug tool merges the per-instance clouds into a single
PointCloud2 whose `intensity` field carries the instance id, so the ROI
extraction output can be inspected in any standard 3D view.

Not launched by any bringup launch file — zero default cost. Run manually:

    ros2 run ai_seg_mask_pointcloud_roi_extractor roi_viz_bridge.py

The output frame is the camera optical frame; in Foxglove either enable
/tf + /tf_static or set the panel's fixed frame to the message frame.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from ai_seg_mask_pointcloud_roi_extractor.msg import ROIPointClouds

_DTYPES = {
    PointField.INT8: 'i1', PointField.UINT8: 'u1',
    PointField.INT16: 'i2', PointField.UINT16: 'u2',
    PointField.INT32: 'i4', PointField.UINT32: 'u4',
    PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
}


def merge(msg, frame_id):
    """Concatenate per-instance clouds; append instance id as intensity."""
    xyz_list, id_list = [], []
    for roi in msg.clouds:
        c = roi.cloud
        if c.width * c.height == 0:
            continue
        fields = {f.name: (f.offset, _DTYPES[f.datatype]) for f in c.fields}
        step = c.point_step
        pts = np.frombuffer(bytes(c.data), dtype=np.uint8).reshape(-1, step)
        xyz = np.zeros((pts.shape[0], 3), dtype=np.float32)
        for i, ax in enumerate(('x', 'y', 'z')):
            off, dt = fields[ax]
            xyz[:, i] = pts[:, off:off + np.dtype(dt).itemsize].copy().view(dt).ravel()
        ok = np.isfinite(xyz).all(axis=1) & ~(xyz == 0).all(axis=1)
        xyz_list.append(xyz[ok])
        id_list.append(np.full(int(ok.sum()), float(roi.id), dtype=np.float32))
    if not xyz_list:
        return None
    xyz = np.concatenate(xyz_list)
    ids = np.concatenate(id_list)
    out = PointCloud2()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = frame_id
    out.height, out.width = 1, xyz.shape[0]
    out.is_dense = True
    out.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    out.point_step = 16
    out.row_step = 16 * out.width
    out.data = np.concatenate([xyz, ids.reshape(-1, 1)], axis=1).tobytes()
    out.is_bigendian = False
    return out


class RoiVizBridge(Node):

    def __init__(self):
        super().__init__('roi_viz_bridge')
        self.declare_parameter('input_topic', '/roi/pointclouds')
        self.declare_parameter('output_topic', '/roi/pointclouds_viz')
        self.declare_parameter('frame_id', '')
        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_override = \
            self.get_parameter('frame_id').get_parameter_value().string_value
        self.frames = 0
        self.points = 0
        self.sub = self.create_subscription(
            ROIPointClouds, in_topic, self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, out_topic, 1)
        self.get_logger().info(
            f'{in_topic} (ROIPointClouds) -> {out_topic} (PointCloud2, '
            f'intensity=instance id)')

    def cb(self, msg):
        if not self.context.ok():
            return
        frame_id = self.frame_override or (msg.header.frame_id or 'camera_optical_frame')
        out = merge(msg, frame_id)
        if out is None:
            return
        self.pub.publish(out)
        self.frames += 1
        self.points += out.width
        if self.frames % 15 == 0:
            self.get_logger().info(
                f'forwarded {self.frames} frames, {self.points} pts total; '
                f'last: {out.width} pts / {len(msg.clouds)} instances, '
                f'frame_id={out.header.frame_id}')


def main(args=None):
    rclpy.init(args=args)
    node = RoiVizBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
