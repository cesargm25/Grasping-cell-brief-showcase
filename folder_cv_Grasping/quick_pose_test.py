#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from yolo_msgs.msg import DetectionArray

import tf2_ros
from tf2_geometry_msgs import do_transform_pose   # IMPORTANT


class CupPoseFromYolo(Node):
    def __init__(self):
        super().__init__('cup_pose_from_yolo')
        
        #do you work ?

        # -------- PARAMETERS --------
        self.declare_parameter('detections_topic', '/yolo/detections_3d')
        self.declare_parameter('target_frame', 'base_link')

        self.det_topic = self.get_parameter('detections_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        # -------- TF BUFFER --------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -------- PUB / SUB --------
        self.pub = self.create_publisher(PoseStamped, '/cup_goal_pose', 10)
        self.sub = self.create_subscription(
            DetectionArray,
            self.det_topic,
            self.cb,
            10
        )

        self.get_logger().info(f'CupPoseFromYolo started. Sub: {self.det_topic}  Target: {self.target_frame}')

    def cb(self, msg: DetectionArray):
        for det in msg.detections:
            if det.class_name != "cup":
                continue

            # --- Pose in YOLO 3D frame (usually camera_link) ---
            pose_cam = PoseStamped()
            pose_cam.header.stamp = msg.header.stamp
            pose_cam.header.frame_id = det.bbox3d.frame_id  # e.g. camera_link

            pose_cam.pose.position.x = float(det.bbox3d.center.position.x)
            pose_cam.pose.position.y = float(det.bbox3d.center.position.y)
            pose_cam.pose.position.z = float(det.bbox3d.center.position.z)
            pose_cam.pose.orientation.w = 1.0

            self.get_logger().info(
                f"[CAMERA FRAME] ({pose_cam.header.frame_id}) "
                f"x={pose_cam.pose.position.x:.3f}, "
                f"y={pose_cam.pose.position.y:.3f}, "
                f"z={pose_cam.pose.position.z:.3f}"
            )

            # --- Transform pose -> base_link (or whatever target_frame is) ---
            try:
                tf_cam = self.tf_buffer.lookup_transform(
                         self.target_frame,          # base_link
                         pose_cam.header.frame_id,   # camera_link
                         rclpy.time.Time(),
                         timeout=Duration(seconds=0.2)
                )
                
                self.get_logger().info(
                    f"[From Camera to Base] camera position in {self.target_frame}: "
                    f"x={tf_cam.transform.translation.x:.3f}, "
                    f"y={tf_cam.transform.translation.y:.3f}, "
                    f"z={tf_cam.transform.translation.z:.3f}"
                )      
             
            except Exception as e:
                self.get_logger().warn(f"Camera TF lookup failed: {e}")
            
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame,            # target
                    pose_cam.header.frame_id,     # source
                    rclpy.time.Time(),            # latest available
                    timeout=Duration(seconds=0.3)
                )

                pose_base_pose = do_transform_pose(pose_cam.pose, tf)

                pose_base = PoseStamped()
                pose_base.header.stamp = msg.header.stamp
                pose_base.header.frame_id = self.target_frame
                pose_base.pose = pose_base_pose

                self.get_logger().info(
                    f"[BASE FRAME] ({pose_base.header.frame_id}) "
                    f"x={pose_base.pose.position.x:.3f}, "
                    f"y={pose_base.pose.position.y:.3f}, "
                    f"z={pose_base.pose.position.z:.3f}"
                )

                self.pub.publish(pose_base)
                return  # only first cup

            except Exception as e:
                self.get_logger().warn(
                    f"TF failed: {e} | source={pose_cam.header.frame_id} target={self.target_frame}"
                )
                return


def main():
    rclpy.init()
    node = CupPoseFromYolo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

