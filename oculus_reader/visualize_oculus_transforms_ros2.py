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

import scipy

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

        self.button_info_init = False
        self.button_triggered_dict = {}
        

    def timer_callback(self):
        while rclpy.ok():
            current_time = self.clock.now()
            self.last_time = current_time
            transformations, buttons = self.oculus_reader.get_transformations_and_buttons()
            if 'r' not in transformations:
                continue
            right_controller_pose = transformations['r']
            left_controller_pose = transformations['l']
            self.publish_transform(right_controller_pose, 'oculus_r')
            self.publish_transform(left_controller_pose, 'oculus_l')

            self.button_funcs(buttons)
            
            self.rate.sleep()

    def button_funcs(self, buttons):
        if not self.button_info_init:
            for key in buttons.keys():
                self.button_triggered_dict[key] = False
            self.button_info_init = True

        # keys: dict_keys([
        ## -- boolean values --
        # 'A',      : Button A on the right controller
        # 'B',      : Button B on the right controller
        # 'RThU',   : Right Thumbstick Up
        # 'RJ',     : Right Joystick Button
        # 'RG',     : Right Grip Button (Bottom trigger)
        # 'RTr',    : Right Trigger Button (Top trigger)
        # 'X',      : Button X on the left controller
        # 'Y',      : Button Y on the left controller
        # 'LThU',   : Left Thumbstick Up
        # 'LJ',     : Left Joystick Button
        # 'LG',     : Left Grip Button (Bottom trigger)
        # 'LTr',   : Left Trigger Button (Top trigger)
        ## -- double values --
        # 'leftJS', : Left Joystick Button (2d as [double, double] for x and y axis)
        # 'leftTrig', : Left Trigger Button (1d as [double] for pressure)
        # 'leftGrip', : Left Grip Button (1d as [double] for pressure)
        # 'rightJS', : Right Joystick Button (2d as [double, double] for x and y axis)
        # 'rightTrig', : Right Trigger Button (1d as [double] for pressure)
        # 'rightGrip' : Right Grip Button (1d as [double] for pressure)
        # ])

        if buttons['A'] and not self.button_triggered_dict['A']:
            self.button_triggered_dict['A'] = True
            if self.tele_cli.wait_for_service(timeout_sec=1.0):
                req = SetBool.Request()
                req.data = True
                result = self.tele_cli.call(req)
                self.get_logger().info(f'Result of service call: {result.success}, message: {result.message}')
            else:
                self.get_logger().error('Service not available')
        if not buttons['A'] and self.button_triggered_dict['A']:
            self.button_triggered_dict['A'] = False

        if buttons['B'] and not self.button_triggered_dict['B']:
            self.button_triggered_dict['B'] = True
            if self.tele_cli.wait_for_service(timeout_sec=1.0):
                req = SetBool.Request()
                req.data = False
                result = self.tele_cli.call(req)
                self.get_logger().info(f'Result of service call: {result.success}, message: {result.message}')
            else:
                self.get_logger().error('Service not available')
        if not buttons['B'] and self.button_triggered_dict['B']:
            self.button_triggered_dict['B'] = False

        if buttons['X'] and not self.button_triggered_dict['X']:
            self.button_triggered_dict['X'] = True
            # for starting a new episode
        if not buttons['X'] and self.button_triggered_dict['X']:
            self.button_triggered_dict['X'] = False

        if buttons['Y'] and not self.button_triggered_dict['Y']:
            self.button_triggered_dict['Y'] = True
            # for stopping and saving current episode
        if not buttons['Y'] and self.button_triggered_dict['Y']:
            self.button_triggered_dict['Y'] = False

        if buttons['LJ'] and not self.button_triggered_dict['LJ']:
            self.button_triggered_dict['LJ'] = True
            # dispose current episode
        if not buttons['LJ'] and self.button_triggered_dict['LJ']:
            self.button_triggered_dict['LJ'] = False

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
        quat = np.array([quat[1], quat[0], -quat[2], quat[3]])
        
        # using scipy rotate quat as x 180 deg and y 90 deg
        r = scipy.spatial.transform.Rotation.from_quat(quat)
        r2 = scipy.spatial.transform.Rotation.from_euler('x', 180, degrees=True)
        r3 = scipy.spatial.transform.Rotation.from_euler('y', -90, degrees=True)
        r_final = r3 * r2 * r
        quat = r_final.as_quat()
       

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
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
