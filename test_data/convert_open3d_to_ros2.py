#!/usr/bin/env python3
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import json
import glob
import os

class Open3DToROS2Publisher(Node):
    def __init__(self, dataset_path):
        super().__init__('open3d_to_ros2')
        self.publisher = self.create_publisher(
            PointCloud2, 
            '/camera/depth/color/points', 
            10
        )
        
        # データセットのパス
        self.depth_files = sorted(glob.glob(os.path.join(dataset_path, 'depth', '*.png')))
        self.color_files = sorted(glob.glob(os.path.join(dataset_path, 'color', '*.png')))
        
        # カメラパラメータ読み込み
        with open(os.path.join(dataset_path, 'camera_intrinsic.json'), 'r') as f:
            intrinsic_data = json.load(f)
        
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.intrinsic.intrinsic_matrix = np.array(intrinsic_data['intrinsic_matrix']).reshape(3, 3)
        
        self.current_frame = 0
        self.timer = self.create_timer(0.1, self.publish_frame)
        
        self.get_logger().info(f'Loaded {len(self.depth_files)} frames')
    
    def publish_frame(self):
        if self.current_frame >= len(self.depth_files):
            self.get_logger().info('All frames published')
            return
        
        # 深度画像とカラー画像読み込み
        depth = o3d.io.read_image(self.depth_files[self.current_frame])
        color = o3d.io.read_image(self.color_files[self.current_frame])
        
        # RGBD画像作成
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, convert_rgb_to_intensity=False
        )
        
        # 点群生成
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, self.intrinsic
        )
        
        # ROS2メッセージに変換
        msg = self.pointcloud_to_ros2(pcd)
        self.publisher.publish(msg)
        
        self.get_logger().info(f'Published frame {self.current_frame}/{len(self.depth_files)}')
        self.current_frame += 1
    
    def pointcloud_to_ros2(self, pcd):
        points = np.asarray(pcd.points)
        
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_depth_optical_frame'
        
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', *point))
        msg.data = b''.join(buffer)
        
        return msg

def main():
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 convert_open3d_to_ros2.py <dataset_path>")
        sys.exit(1)
    
    rclpy.init()
    node = Open3DToROS2Publisher(sys.argv[1])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
