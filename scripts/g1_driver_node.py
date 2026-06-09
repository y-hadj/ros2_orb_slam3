#!/usr/bin/env python3
"""
G1 D435i Driver Node for ros2_orb_slam3.
Reads ROS1 bag and publishes ROS2 topics for ORB-SLAM3 using custom handshake protocol.
"""

import sys
import argparse
import cv2
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String, Float64
import std_msgs.msg


class G1DriverNode(Node):
    def __init__(self, bag_path, mode):
        super().__init__('g1_driver_node')
        
        self.mode = mode
        self.typestore = get_typestore(Stores.ROS1_NOETIC)
        
        # Publishers
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        self.pub_imu_to_agent_name = "/mono_py_driver/imu_msg"
        
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)
        self.subscribe_exp_ack_ = self.create_subscription(String, self.sub_exp_ack_name, self.ack_callback, 10)
        self.publish_img_msg_ = self.create_publisher(Image, self.pub_img_to_agent_name, 1)
        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_to_agent_name, 1)
        self.publish_imu_msg_ = self.create_publisher(Imu, self.pub_imu_to_agent_name, 100)
        
        self.send_config = True
        self.settings_name = "G1"
        
        # Read bag
        self.bag_path = Path(bag_path).expanduser()
        self.frames = []
        self.imu_buffer = []
        self.read_bag()
        
        self.frame_idx = 0
        
    def read_bag(self):
        frames = {}
        imu_all = []
        
        with AnyReader([self.bag_path], default_typestore=self.typestore) as reader:
            for connection, timestamp, rawdata in reader.messages():
                topic = connection.topic
                msg = reader.deserialize(rawdata, connection.msgtype)
                ts = round(timestamp * 1e-9, 3)
                
                if ts not in frames:
                    frames[ts] = {"image": None, "imu": []}
                
                if topic == "/camera/color/image_raw/compressed" and self.mode in ["mono", "mono_inertial"]:
                    frames[ts]["image"] = msg
                elif topic == "/camera/imu":
                    frames[ts]["imu"].append(msg)
                    imu_all.append((ts, msg))
        
        self.frames = [
            (ts, f["image"], f["imu"]) 
            for ts, f in sorted(frames.items()) 
            if f["image"] is not None
        ]
        
        self.imu_all = imu_all
        self.get_logger().info(f"Loaded {len(self.frames)} frames and {len(imu_all)} IMU readings from bag")
    
    def ack_callback(self, msg):
        self.get_logger().info(f"Got ack: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False
    
    def handshake_with_cpp_node(self):
        if self.send_config:
            msg = String()
            msg.data = self.settings_name
            self.publish_exp_config_.publish(msg)
    
    def publish_imu(self, imu_msg_ros, timestamp):
        imu = Imu()
        imu.header.stamp.sec = int(timestamp)
        imu.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
        imu.header.frame_id = "imu"
        imu.angular_velocity.x = imu_msg_ros.angular_velocity.x
        imu.angular_velocity.y = imu_msg_ros.angular_velocity.y
        imu.angular_velocity.z = imu_msg_ros.angular_velocity.z
        imu.linear_acceleration.x = imu_msg_ros.linear_acceleration.x
        imu.linear_acceleration.y = imu_msg_ros.linear_acceleration.y
        imu.linear_acceleration.z = imu_msg_ros.linear_acceleration.z
        self.publish_imu_msg_.publish(imu)
    
    def publish_frame(self, idx):
        if idx >= len(self.frames):
            self.get_logger().info("Bag playback complete")
            return False
        
        ts, img_msg, imu_msgs = self.frames[idx]
        
        # Publish all IMU readings for this frame
        for imu_msg in imu_msgs:
            self.publish_imu(imu_msg, ts)
        
        # Decompress and publish image
        if img_msg is not None:
            np_arr = np.frombuffer(img_msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            h, w = cv_img.shape[:2]
            img_msg_ros = Image()
            img_msg_ros.header.stamp.sec = int(ts)
            img_msg_ros.header.stamp.nanosec = int((ts - int(ts)) * 1e9)
            img_msg_ros.header.frame_id = "camera"
            img_msg_ros.height = h
            img_msg_ros.width = w
            img_msg_ros.encoding = "bgr8"
            img_msg_ros.is_bigendian = 0
            img_msg_ros.step = w * 3
            img_msg_ros.data = cv_img.tobytes()
            
            timestep_msg = Float64()
            timestep_msg.data = float(ts)
            
            self.publish_timestep_msg_.publish(timestep_msg)
            self.publish_img_msg_.publish(img_msg_ros)
        
        return True


def main():
    parser = argparse.ArgumentParser(description="G1 Driver Node for ORB-SLAM3")
    parser.add_argument("--bag", required=True, help="Path to ROS1 bag file")
    parser.add_argument("--mode", required=True, choices=["mono", "mono_inertial", "stereo"], help="SLAM mode")
    args = parser.parse_args()
    
    if args.mode == "stereo":
        print("Support for stereo is not available yet")
        sys.exit(1)
    
    rclpy.init()
    node = G1DriverNode(args.bag, args.mode)
    
    rate = node.create_rate(30)
    
    # Handshake loop
    while node.send_config:
        node.handshake_with_cpp_node()
        rclpy.spin_once(node)
    
    print("Handshake complete")
    
    # Publish frames
    for idx in range(len(node.frames)):
        rclpy.spin_once(node)
        if not node.publish_frame(idx):
            break
        rate.sleep()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
