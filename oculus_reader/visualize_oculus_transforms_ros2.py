from reader  import OculusReader
from tf_transformations import quaternion_from_matrix
import rclpy
from rclpy.node import Node
from rclpy.timer import Rate
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
from rclpy import time, clock

from std_srvs.srv import SetBool
from example_interfaces.srv import AddTwoInts

# For spinning
from rclpy.executors import SingleThreadedExecutor
from threading import Thread

class OculusReaderNode(Node):
    def __init__(self):
        super().__init__('oculus_reader')
        self.oculus_reader = OculusReader()
        self.br = tf2_ros.TransformBroadcaster(self)

        self.clock = rclpy.clock.Clock()
        self.last_time = self.clock.now()
        self.last_x_r = 0.0
        self.last_x_r_different_time = self.clock.now()

        self.tele_cli = self.create_client(SetBool, '/teleop_start')
        self.button_count_poll= 0

        # Create a thread to run the main loop
        self.rate = self.create_rate(100)  # 90 Hz
        self.thread = Thread(target=self.timer_callback)
        self.thread.start()
        

    def timer_callback(self):
        while rclpy.ok():
            current_time = self.clock.now()
            # self.get_logger().info(f'Delta time: {(current_time - self.last_time).nanoseconds / 1e6} ms')
            self.last_time = current_time
            transformations, buttons = self.oculus_reader.get_transformations_and_buttons()
            if 'r' not in transformations:
                continue
            right_controller_pose = transformations['r']
            left_controller_pose = transformations['l']
            self.publish_transform(right_controller_pose, 'oculus_r')
            self.publish_transform(left_controller_pose, 'oculus_l')
            # self.get_logger().info(f'transformations: {transformations}')
            # self.get_logger().info(f'buttons: {buttons}')

            
            self.button_count_poll += 1
            if self.button_count_poll >= 10:
                self.button_count_poll = 0
                self.button_funcs(buttons)
            
            # self.get_logger().info(f'looping')
            self.rate.sleep()

    def button_funcs(self, buttons):
        if buttons['A']:
            if self.tele_cli.wait_for_service(timeout_sec=1.0):
                req = SetBool.Request()
                req.data = True
                result = self.tele_cli.call(req)
                self.get_logger().info(f'Result of service call: {result.success}, message: {result.message}')
            else:
                self.get_logger().error('Service not available')
        
        if buttons['B']:
            if self.tele_cli.wait_for_service(timeout_sec=1.0):
                req = SetBool.Request()
                req.data = False
                result = self.tele_cli.call(req)
                self.get_logger().info(f'Result of service call: {result.success}, message: {result.message}')
            else:
                self.get_logger().error('Service not available')

    def publish_transform(self, transform, name):
        translation = transform[:3, 3]
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'oculus_base'
        t.child_frame_id = name
        t.transform.translation.x = -translation[2]
        t.transform.translation.y = -translation[0]
        t.transform.translation.z = translation[1]
        quat = quaternion_from_matrix(transform)
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[0]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OculusReaderNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
