from reader  import OculusReader
from tf_transformations import quaternion_from_matrix
import rclpy
from rclpy.node import Node
from rclpy.timer import Rate
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
from rclpy import time, clock
from std_srvs.srv import SetBool,Trigger
from std_msgs.msg import Float64MultiArray

# For spinning
from rclpy.executors import SingleThreadedExecutor
from threading import Thread

import scipy
from scipy.spatial.transform import Rotation

class OculusReaderNode(Node):
    def __init__(self):
        super().__init__('oculus_reader')
        self.oculus_reader = OculusReader()
        self.br = tf2_ros.TransformBroadcaster(self)

        self.clock = rclpy.clock.Clock()
        self.last_time = self.clock.now()

        # subscriber
        self.create_subscription(Float64MultiArray, '/L_haptic_amplitude', self.left_haptic_callback, 10)
        self.create_subscription(Float64MultiArray, '/R_haptic_amplitude', self.right_haptic_callback, 10)

        # publisher
        self.robotiq_l = self.create_publisher(Float64MultiArray, '/L_gripper_forward_position_controller/commands', 10)
        self.robotiq_r = self.create_publisher(Float64MultiArray, '/R_gripper_forward_position_controller/commands', 10)

        # service
        self.tele_cli = self.create_client(SetBool, '/teleop_start')
        self.dataset_record_cli = self.create_client(Trigger, '/start_recording')
        self.dataset_stop_cli = self.create_client(SetBool, '/stop_recording')
        self.phase_state_cli = self.create_client(Trigger, '/stage_up')
        self.button_count_poll= 0
        self.button_info_init = False
        self.button_triggered_dict = {}

        # Create a thread to run the main loop
        self.rate = self.create_rate(70)  # 90 Hz
        self.thread = Thread(target=self.loop)
        self.thread.start()

        

    def loop(self):
        while rclpy.ok():
            current_time = self.clock.now()
            self.last_time = current_time
            transformations, buttons = self.oculus_reader.get_transformations_and_buttons()

            self.button_funcs(buttons)
            if 'r' not in transformations:
                continue
            if 'l' not in transformations:
                continue
            
            

            right_controller_pose = transformations['r']
            left_controller_pose = transformations['l']
            tf_l, valid_l = self.publish_transform(right_controller_pose, 'oculus_r')
            tf_r, valid_r = self.publish_transform(left_controller_pose, 'oculus_l')

            if valid_l and valid_r:
                self.br.sendTransform(tf_l)
                self.br.sendTransform(tf_r)          
            else:
                self.get_logger().warning('Invalid transform detected, not publishing TF.')  
            self.rate.sleep()

    def button_funcs(self, buttons):
        # early return if no A or X button info
        if 'A' not in buttons:
            return
        if 'X' not in buttons:
            return
        
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
        elif not buttons['A'] and self.button_triggered_dict['A']:
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
        elif not buttons['B'] and self.button_triggered_dict['B']:
            self.button_triggered_dict['B'] = False

        if buttons['X'] and not self.button_triggered_dict['X']:
            self.button_triggered_dict['X'] = True
            # for starting a new episode
            if self.dataset_record_cli.wait_for_service(timeout_sec=1.0):
                req = Trigger.Request()
                result = self.dataset_record_cli.call(req)
                self.get_logger().info(f'Result of service call: {result.success}, message: {result.message}')
            else:
                self.get_logger().error('Service not available')
        elif not buttons['X'] and self.button_triggered_dict['X']:
            self.button_triggered_dict['X'] = False

        if buttons['Y'] and not self.button_triggered_dict['Y']:
            self.button_triggered_dict['Y'] = True
            # for stopping and saving current episode
            if self.dataset_stop_cli.wait_for_service(timeout_sec=1.0):
                req = SetBool.Request()
                req.data = True
                result = self.dataset_stop_cli.call(req)
                self.get_logger().info(f'Result of service call: {result.success}, message: {result.message}')
            else:
                self.get_logger().error('Service not available')
        elif not buttons['Y'] and self.button_triggered_dict['Y']:
            self.button_triggered_dict['Y'] = False

        if buttons['LJ'] and not self.button_triggered_dict['LJ']:
            self.button_triggered_dict['LJ'] = True
            # dispose current episode
            if self.dataset_stop_cli.wait_for_service(timeout_sec=1.0):
                req = SetBool.Request()
                req.data = False
                result = self.dataset_stop_cli.call(req)
                self.get_logger().info(f'Result of service call: {result.success}, message: {result.message}')
            else:
                self.get_logger().error('Service not available')
        elif not buttons['LJ'] and self.button_triggered_dict['LJ']:
            self.button_triggered_dict['LJ'] = False

        # gripper control
        if buttons['LTr'] and not self.button_triggered_dict['LTr']:
            self.button_triggered_dict['LTr'] = True
            msg = Float64MultiArray()
            msg.data = [0.]  # close
            self.robotiq_l.publish(msg)
        elif not buttons['LTr'] and self.button_triggered_dict['LTr']:
            self.button_triggered_dict['LTr'] = False

        if buttons['LG'] and not self.button_triggered_dict['LG']:
            self.button_triggered_dict['LG'] = True

        elif not buttons['LG'] and self.button_triggered_dict['LG']:
            self.button_triggered_dict['LG'] = False

        if buttons['RTr'] and not self.button_triggered_dict['RTr']:
            self.button_triggered_dict['RTr'] = True
            msg = Float64MultiArray()
            msg.data = [0.]  # close
            self.robotiq_r.publish(msg)
        elif not buttons['RTr'] and self.button_triggered_dict['RTr']:
            self.button_triggered_dict['RTr'] = False

        if buttons['RG'] and not self.button_triggered_dict['RG']:
            self.button_triggered_dict['RG'] = True
            
        elif not buttons['RG'] and self.button_triggered_dict['RG']:
            self.button_triggered_dict['RG'] = False

        if buttons['rightJS'][0] > 0.8 and not self.button_triggered_dict['rightJS']:
            self.button_triggered_dict['rightJS'] = True
            # phase up
            if self.phase_state_cli.wait_for_service(timeout_sec=1.0):
                req = Trigger.Request()
                result = self.phase_state_cli.call(req)
                self.get_logger().info(f'Result of service call: {result.success}, message: {result.message}')
            else:
                self.get_logger().error('Service not available')
        elif buttons['rightJS'][0] < 0.2 and self.button_triggered_dict['rightJS']:
            self.button_triggered_dict['rightJS'] = False

        # Using LeftGrasp control left gripper to  decrease accidental triggering
        if buttons['leftGrip'][0] > 0.99 and not self.button_triggered_dict['leftGrip']:
            self.button_triggered_dict['leftGrip'] = True
            msg = Float64MultiArray()
            msg.data = [0.8]  # close
            self.robotiq_l.publish(msg)
        elif buttons['leftGrip'][0] < 0.2 and self.button_triggered_dict['leftGrip']:
            self.button_triggered_dict['leftGrip'] = False
        
        if buttons['rightGrip'][0] > 0.99 and not self.button_triggered_dict['rightGrip']:
            self.button_triggered_dict['rightGrip'] = True
            msg = Float64MultiArray()
            msg.data = [0.8]  # close
            self.robotiq_r.publish(msg)
        elif buttons['rightGrip'][0] < 0.2 and self.button_triggered_dict['rightGrip']:
            self.button_triggered_dict['rightGrip'] = False


    def publish_transform(self, transform, name):
        translation = transform[:3, 3]
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'oculus_base'
        t.child_frame_id = name
        t.transform.translation.x = -translation[2]
        t.transform.translation.y = -translation[0]
        t.transform.translation.z = translation[1]

        rot_m = np.array([[transform[0][0], transform[0][1], transform[0][2],],
                          [transform[1][0], transform[1][1], transform[1][2],],
                          [transform[2][0], transform[2][1], transform[2][2]]])
        if abs(rot_m.sum()) < 1e-6:
            return t, False
        quat = quaternion_from_matrix(transform)
        quat = np.array([quat[1], quat[0], -quat[2], quat[3]])
        
        # using scipy rotate quat as x 180 deg and y 90 deg
        r = Rotation.from_quat(quat)
        r2 = Rotation.from_euler('x', 180, degrees=True)
        r3 = Rotation.from_euler('y', -90, degrees=True)
        r_final = r3 * r2 * r
        quat = r_final.as_quat()
       

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        return t, True

    def left_haptic_callback(self, msg):
        intensity = msg.data[0]
        self.oculus_reader.set_haptic_left(intensity)
    
    def right_haptic_callback(self, msg):
        intensity = msg.data[0]
        self.oculus_reader.set_haptic_right(intensity)

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
