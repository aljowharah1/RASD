#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import struct


class ROIFilter(Node):
    def __init__(self):
        super().__init__('roi_filter_node')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/roi_point_cloud')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.callback,
            rclpy.qos.QoSProfile(depth=5)
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.get_logger().info("ROI filter READY (TF enabled)")


    def callback(self, msg):

        # ================================
        # 1) TRANSFORM POINT CLOUD
        # ================================
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", msg.header.frame_id, rclpy.time.Time()
            )
            msg = do_transform_cloud(msg, transform)

        except Exception as e:
            self.get_logger().warn(f"[TF ERROR] Failed TF transform: {e}")
            return

        self.get_logger().info("✓ TF applied successfully")


        # ================================
        # 2) READ RAW POINTS
        # ================================
        pts = []
        for x, y, z, intensity in pc2.read_points(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
        ):
            pts.append((x, y, z, intensity))

        if len(pts) == 0:
            self.get_logger().warn("⚠ RAW cloud is EMPTY after reading points.")
            return

        self.get_logger().info(f"RAW count: {len(pts)}")


        pts = np.array(pts, dtype=np.float32)

        # Print FIRST 5 points
        self.get_logger().info(f"Sample points: {pts[:5]}")


        # ================================
        # 3) XY CORRIDOR FILTER
        # ================================
        mask_xy = (
            (pts[:, 0] >= 0.0) &
            (pts[:, 0] <= 60.0) &
            (np.abs(pts[:, 1]) <= 2.0)
        )
        pts_xy = pts[mask_xy]

        self.get_logger().info(f"After XY filter: {pts_xy.shape[0]} points")

        if pts_xy.shape[0] == 0:
            self.get_logger().warn("⚠ XY filter removed ALL points")
            return


        # ================================
        # 4) Z LIMIT / HEIGHT FILTER
        # ================================
        mask_z = (pts_xy[:, 2] >= -2.0) & (pts_xy[:, 2] <= 2.0)
        pts_final = pts_xy[mask_z]

        self.get_logger().info(f"After Z filter: {pts_final.shape[0]} points")

        if pts_final.shape[0] == 0:
            self.get_logger().warn("⚠ Z filter removed ALL points")
            return


        # ================================
        # 5) GROUND DEBUGGING PRINT
        # ================================
        avg_ground = np.mean(pts_final[:, 2])
        min_ground = np.min(pts_final[:, 2])
        max_ground = np.max(pts_final[:, 2])

        self.get_logger().info(
            f"GROUND DEBUG: mean_z={avg_ground:.3f}, "
            f"min_z={min_ground:.3f}, max_z={max_ground:.3f}"
        )


        # ================================
        # 6) BUILD OUTPUT POINTCLOUD
        # ================================
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = "base_link"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        buf = b''.join(
            struct.pack('ffff', float(x), float(y), float(z), float(i))
            for (x, y, z, i) in pts_final
        )

        out = PointCloud2()
        out.header = header
        out.height = 1
        out.width = pts_final.shape[0]
        out.fields = fields
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = 16 * out.width
        out.is_dense = True
        out.data = buf

        self.pub.publish(out)

        self.get_logger().info(f"Published ROI cloud with {pts_final.shape[0]} points")


def main(args=None):
    rclpy.init(args=args)
    node = ROIFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

