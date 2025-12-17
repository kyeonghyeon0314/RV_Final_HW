#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import open3d as o3d
import numpy as np
import struct
import os
import collections

# =============================================================================
# COLMAP Binary parsing helper functions
# =============================================================================

BaseImage = collections.namedtuple(
    "Image", ["id", "qvec", "tvec", "camera_id", "name", "xys", "point3D_ids"])

def qvec2rotmat(qvec):
    return np.array([
        [1 - 2 * qvec[2]**2 - 2 * qvec[3]**2,
         2 * qvec[1] * qvec[2] - 2 * qvec[0] * qvec[3],
         2 * qvec[3] * qvec[1] + 2 * qvec[0] * qvec[2]],
        [2 * qvec[1] * qvec[2] + 2 * qvec[0] * qvec[3],
         1 - 2 * qvec[1]**2 - 2 * qvec[3]**2,
         2 * qvec[2] * qvec[3] - 2 * qvec[0] * qvec[1]],
        [2 * qvec[3] * qvec[1] - 2 * qvec[0] * qvec[2],
         2 * qvec[2] * qvec[3] + 2 * qvec[0] * qvec[1],
         1 - 2 * qvec[1]**2 - 2 * qvec[2]**2]])

def read_next_bytes(fid, num_bytes, format_char_sequence, endian_character="<"):
    data = fid.read(num_bytes)
    return struct.unpack(endian_character + format_char_sequence, data)

def read_images_binary(path_to_model_file):
    images = {}
    with open(path_to_model_file, "rb") as fid:
        num_reg_images = read_next_bytes(fid, 8, "Q")[0]
        for _ in range(num_reg_images):
            binary_image_properties = read_next_bytes(fid, 64, "idddddddi")
            image_id = binary_image_properties[0]
            qvec = np.array(binary_image_properties[1:5])
            tvec = np.array(binary_image_properties[5:8])
            camera_id = binary_image_properties[8]
            image_name = ""
            current_char = read_next_bytes(fid, 1, "c")[0]
            while current_char != b"\x00":
                image_name += current_char.decode("utf-8")
                current_char = read_next_bytes(fid, 1, "c")[0]
            
            num_points2D = read_next_bytes(fid, 8, "Q")[0]
            # Skip 2D points (x, y, point3D_id) -> 24 bytes per point
            fid.seek(24 * num_points2D, 1)
            
            images[image_id] = BaseImage(
                id=image_id, qvec=qvec, tvec=tvec,
                camera_id=camera_id, name=image_name,
                xys=None, point3D_ids=None)
    return images

# =============================================================================
# ROS Node
# =============================================================================

class ColmapPublisher(Node):
    def __init__(self):
        super().__init__('colmap_data_publisher')
        
        # Publishers
        # PointCloud2 for the dense model
        self.pcd_pub = self.create_publisher(PointCloud2, '/colmap/points', 10)
        # MarkerArray for camera poses (trajectory)
        self.marker_pub = self.create_publisher(MarkerArray, '/colmap/camera_poses', 10)
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Paths (USER: Check these paths)
        self.base_path = "/home/kimkh/RV_Final_HW"
        self.ply_path = os.path.join(self.base_path, "dense/0/fused.ply")
        self.images_bin_path = os.path.join(self.base_path, "sparse/0/images.bin")
        
        # Data containers
        self.pcd_msg = None
        self.marker_array_msg = None
        
        # Load data on startup
        self.load_point_cloud()
        self.load_camera_poses()

    def load_point_cloud(self):
        self.get_logger().info(f"Loading Point Cloud from {self.ply_path}...")
        if not os.path.exists(self.ply_path):
            self.get_logger().error(f"File not found: {self.ply_path}")
            return

        try:
            pcd = o3d.io.read_point_cloud(self.ply_path)
            if not pcd.has_points():
                self.get_logger().warn("Point cloud is empty.")
                return
            
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors)
            if colors.max() <= 1.0:
                colors = (colors * 255).astype(np.uint8)
            
            self.pcd_msg = self.create_pcd_msg(points, colors)
            self.get_logger().info(f"Loaded {len(points)} points.")
        except Exception as e:
            self.get_logger().error(f"Failed to load PCD: {e}")

    def create_pcd_msg(self, points, colors):
        msg = PointCloud2()
        msg.header.frame_id = "map"
        msg.height = 1
        msg.width = len(points)
        
        # 'rgb' field is a float32 but contains packed RGB data
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        
        buffer = []
        for i in range(len(points)):
            x, y, z = points[i]
            r, g, b = colors[i]
            # Pack r,g,b into a single 32-bit integer, then reinterpret as float
            rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
            rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
            
            buffer.append(struct.pack('ffff', x, y, z, rgb_float))
            
        msg.data = b"".join(buffer)
        return msg

    def load_camera_poses(self):
        self.get_logger().info(f"Loading Camera Poses from {self.images_bin_path}...")
        if not os.path.exists(self.images_bin_path):
            self.get_logger().error(f"File not found: {self.images_bin_path}")
            return

        try:
            images = read_images_binary(self.images_bin_path)
            sorted_image_ids = sorted(images.keys())
            
            marker_array = MarkerArray()
            
            # 1. Line Strip (Trajectory)
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.ns = "trajectory"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05
            line_marker.color.a = 1.0
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0 # Yellow path
            
            # 2. Camera Objects (Arrows)
            for i, img_id in enumerate(sorted_image_ids):
                img = images[img_id]
                
                # Camera Center C = -R^t * t
                R = qvec2rotmat(img.qvec)
                t = img.tvec
                C = -R.T @ t
                
                # Add point to trajectory line
                p = Point()
                p.x, p.y, p.z = float(C[0]), float(C[1]), float(C[2])
                line_marker.points.append(p)
                
                # Create Camera Arrow Marker
                arrow = Marker()
                arrow.header.frame_id = "map"
                arrow.ns = "cameras"
                arrow.id = int(img_id)
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.scale.x = 0.3  # Length
                arrow.scale.y = 0.05 # Width
                arrow.scale.z = 0.05 # Height
                arrow.color.a = 1.0
                arrow.color.r = 0.0
                arrow.color.g = 1.0
                arrow.color.b = 0.0 # Green arrows
                
                arrow.pose.position = p
                
                # Orientation: Convert World-to-Camera Quaternion to Camera-to-World
                # Conjugate of unit quaternion [w, x, y, z] is [w, -x, -y, -z]
                arrow.pose.orientation.w = img.qvec[0]
                arrow.pose.orientation.x = -img.qvec[1]
                arrow.pose.orientation.y = -img.qvec[2]
                arrow.pose.orientation.z = -img.qvec[3]
                
                marker_array.markers.append(arrow)
            
            marker_array.markers.append(line_marker)
            self.marker_array_msg = marker_array
            self.get_logger().info(f"Loaded {len(sorted_image_ids)} camera poses.")

        except Exception as e:
            self.get_logger().error(f"Failed to load Camera Poses: {e}")

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        
        # Publish Point Cloud
        if self.pcd_msg:
            self.pcd_msg.header.stamp = now
            self.pcd_pub.publish(self.pcd_msg)
            
        # Publish Camera Poses
        if self.marker_array_msg:
            for m in self.marker_array_msg.markers:
                m.header.stamp = now
            self.marker_pub.publish(self.marker_array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ColmapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
