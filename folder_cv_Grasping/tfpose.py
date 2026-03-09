#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped # From Huy’s code
from yolo_msgs.msg import DetectionArray

import tf2_ros
from tf2_ros import TransformBroadcaster # From Huy’s code
from tf2_geometry_msgs import do_transform_pose  # works on geometry_msgs/Pose


class CupPoseFromYolo(Node):
    def __init__(self):
        super().__init__('cup_pose_from_yolo')

        # Paramaters 
        self.declare_parameter('detections_topic', '/yolo/detections_3d') # This one is supposed to be camer_color_optical_frame
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('depth_frame', 'camera_depth_optical_frame')
        self.declare_parameter('color_link_frame', 'camera_color_optical_frame') # Technically this one must 
        #give us the same value as detections_topic

        # frame name to test the "what if bbox3d was camera_link?"
        self.declare_parameter('camera_link_frame', 'camera_link')

        self.det_topic = self.get_parameter('detections_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.depth_frame = self.get_parameter('depth_frame').value
        self.camera_link_frame = self.get_parameter('camera_link_frame').value # Test 1, works in simulation
        self.color_link_frame = self.get_parameter('color_link_frame').value # Test 2, test1 did not work on reality
        

        # TF Buffer 
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0)) 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TF Brocaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Pub & Sub 
        self.pub = self.create_publisher(PoseStamped, '/cup_goal_pose', 10)
        self.sub = self.create_subscription(
            DetectionArray,
            self.det_topic,
            self.cb,
            10
        )

        self.get_logger().info(
            f'CupPoseFromYolo started. Sub: {self.det_topic} '
            f'Target: {self.target_frame} DepthFrame: {self.depth_frame} '
            f'CameraLinkFrame: {self.camera_link_frame}'
            f'ColorLinkFrame: {self.color_link_frame}'
        )

    def _transform_pose_stamped(self, pose_in: PoseStamped, target_frame: str, tf_time, timeout_sec: float = 0.3) -> PoseStamped:
        
        # Transform PoseStamped pose_in to target_frame using lookup_transform and do_transform_pose(Pose)
        # This avoids tf_buffer.transform() type support issues
        
        tf = self.tf_buffer.lookup_transform(
            target_frame,                 # target
            pose_in.header.frame_id,      # source
            tf_time,
            timeout=Duration(seconds=timeout_sec)
        )

        pose_out = PoseStamped()
        pose_out.header.stamp = pose_in.header.stamp
        pose_out.header.frame_id = target_frame
        pose_out.pose = do_transform_pose(pose_in.pose, tf)  # returns geometry_msgs/Pose
        return pose_out

    def cb(self, msg: DetectionArray):
        tf_time = rclpy.time.Time.from_msg(msg.header.stamp)

        for det in msg.detections:
            if det.class_name != "cup":
                continue

    
            # Pose as reported by YOLO, Given from the camera frame
         
            pose_cam = PoseStamped()
            pose_cam.header.stamp = msg.header.stamp
            pose_cam.header.frame_id = det.bbox3d.frame_id  # e.g. camera_color_optical_frame

            pose_cam.pose.position.x = float(det.bbox3d.center.position.x)
            pose_cam.pose.position.y = float(det.bbox3d.center.position.y)
            pose_cam.pose.position.z = float(det.bbox3d.center.position.z)
            pose_cam.pose.orientation.w = 1.0

            self.get_logger().info(
                f"[CUp position Camera] ({pose_cam.header.frame_id}) "
                f"x={pose_cam.pose.position.x:.3f}, "
                f"y={pose_cam.pose.position.y:.3f}, "
                f"z={pose_cam.pose.position.z:.3f}"
            )

            try:
                
                # Convert RAW to DEPTH optical frame
             
                if pose_cam.header.frame_id != self.depth_frame:
                    pose_cam_depth = self._transform_pose_stamped(
                        pose_cam, self.depth_frame, tf_time, timeout_sec=0.3
                    )
                else:
                    pose_cam_depth = pose_cam

                self.get_logger().info(
                    f"[Cup position camera 2] ({pose_cam_depth.header.frame_id}) "
                    f"x={pose_cam_depth.pose.position.x:.3f}, "
                    f"y={pose_cam_depth.pose.position.y:.3f}, "
                    f"z={pose_cam_depth.pose.position.z:.3f}"
                )

                # camera depth frame origin in base_link
                tf_cam = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    pose_cam_depth.header.frame_id,
                    tf_time,
                    timeout=Duration(seconds=0.3)
                )
                self.get_logger().info(
                    f"[Camera position from base_link] {self.target_frame}: "
                    f"x={tf_cam.transform.translation.x:.3f}, "
                    f"y={tf_cam.transform.translation.y:.3f}, "
                    f"z={tf_cam.transform.translation.z:.3f}"
                )

                
                # Convert DEPTH to  BASE  
                pose_base = self._transform_pose_stamped(
                    pose_cam_depth, self.target_frame, tf_time, timeout_sec=0.3
                )

                self.get_logger().info(
                    f"[1. BASE FRAME] ({self.target_frame}) "
                    f"x={pose_base.pose.position.x:.3f}, "
                    f"y={pose_base.pose.position.y:.3f}, "
                    f"z={pose_base.pose.position.z:.3f}"
                )

                # -------------------------
                # What if the SAME XYZ is actually in camera_link?
            
                
                pose_as_camlink = PoseStamped()
                pose_as_camlink.header.stamp = msg.header.stamp
                pose_as_camlink.header.frame_id = self.camera_link_frame

                # Copy the same numeric values
                pose_as_camlink.pose.position.x = pose_cam.pose.position.x
                pose_as_camlink.pose.position.y = pose_cam.pose.position.y
                pose_as_camlink.pose.position.z = pose_cam.pose.position.z
                pose_as_camlink.pose.orientation.w = 1.0

                pose_base_from_camlink = self._transform_pose_stamped(
                    pose_as_camlink, self.target_frame, tf_time, timeout_sec=0.3
                )

                self.get_logger().info(
                    f"[Mug position] ({self.target_frame}) "
                    f"x={pose_base_from_camlink.pose.position.x:.3f}, "
                    f"y={pose_base_from_camlink.pose.position.y:.3f}, "
                    f"z={pose_base_from_camlink.pose.position.z:.3f} "
                )

                # Just added, This is to get pose and transfor to be taken as frame ok Rviz, just to check

                t = TransformStamped()

                # same timestamp transformaded pose 
                t.header.stamp = pose_base_from_camlink.header.stamp
                
                
                t.header.frame_id = self.target_frame   # "base_link"
                
                t.child_frame_id = "cup" # Label
                t.transform.translation.x = pose_base_from_camlink.pose.position.x
                t.transform.translation.y = pose_base_from_camlink.pose.position.y
                t.transform.translation.z = pose_base_from_camlink.pose.position.z
                
                # had to assume ratation value 
                
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                
                self.tf_broadcaster.sendTransform(t)


         #       # ---------------------------------- Test to verify the 
#
         #       pose_as_colrlink = PoseStamped()
         #       pose_as_colrlink.header.stamp = msg.header.stamp
         #       pose_as_colrlink.header.frame_id = self.color_link_frame
#
         #       # Copy the same numeric values
         #       pose_as_colrlink.pose.position.x = pose_cam.pose.position.x
         #       pose_as_colrlink.pose.position.y = pose_cam.pose.position.y
         #       pose_as_colrlink.pose.position.z = pose_cam.pose.position.z
         #       pose_as_colrlink.pose.orientation.w = 1.0
#
         #       pose_base_from_colrlink = self._transform_pose_stamped(
         #           pose_as_colrlink, self.target_frame, tf_time, timeout_sec=0.3
         #       )
#
         #       self.get_logger().info(
         #           f"[2. CUP C. FROM BASE COLOR_OPTICAL] ({self.target_frame}) "
         #           f"x={pose_base_from_colrlink.pose.position.x:.3f}, "
         #           f"y={pose_base_from_colrlink.pose.position.y:.3f}, "
         #           f"z={pose_base_from_colrlink.pose.position.z:.3f} "
         #           #f"| assumed_source={self.camera_link_frame}"
                #)

                #----------------------------------- End of test...

                # Publish the original pose_base (unchanged behavior)
                #self.pub.publish(pose_base)
                self.pub.publish(pose_base_from_camlink)
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
