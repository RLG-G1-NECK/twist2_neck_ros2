import argparse
import json
from time import sleep

import rclpy
from scipy.spatial.transform import Rotation

from typing import get_args

from rclpy.node import Node, Parameter, ParameterDescriptor, Timer
from twist2_neck_ros2.neck_controller.neck_controller import (
    NeckController,
    MotorConfig,
    MotorType,
)
from motion_smoothener.smoothen_motion import Smoothener

from tf2_ros import TransformException, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class NeckNode(Node):
    """
    This node listens for a head transform and controls the TWIST2 Neck using
    the neck_controller library.
    """
    def __init__(
            self, 
            device_name: str, device_baudrate: int, 
            motors_config: dict[MotorType, MotorConfig]
        ):
        """
        Create a new neck node.
        
        :param str device_name:     The name of the serial device or port
        :param str device_baudrate: The baudrate of the device or port
        :param dict[MotorType, MotorConfig] motors_config:
                                    A dictionary containing the configurations of the yaw
                                    and pitch motor, with 'yaw' and 'pitch' as keys and 
                                    MotorConfig as values
        """
        super().__init__('neck_node')

        # create controller
        self.controller: NeckController = NeckController(
            device_name, device_baudrate,
            motors_config['yaw'], motors_config['pitch'])


        # get node parameters
        self._declare_parameters()
        self.head_frame: str = self.get_parameter('head_frame').get_parameter_value().string_value
        self.world_frame: str = self.get_parameter('world_frame').get_parameter_value().string_value
        poll_period: float = 1.0 / self.get_parameter('frequency').get_parameter_value().integer_value

        self.tf_buffer: Buffer = Buffer()
        self.tf_listening: TransformListener = TransformListener(self.tf_buffer, self)

        self.timer: Timer = self.create_timer(poll_period, self._move_head)
        self.shutting_down = False

        # NEW
        self.pitch_smoothener = Smoothener(max_d2_per_step="max_pitch_d2")
        self.yaw_smoothener = Smoothener(max_d2_per_step="max_yaw_d2")


    def _declare_parameters(self):
        """Declare all parameters of this node."""
        self.declare_parameter(
            'head_frame', 'head',
            ParameterDescriptor(
                type=Parameter.Type.STRING,
                description=('The name of the frame of the head.')))
        self.declare_parameter(
            'world_frame', 'world',
            ParameterDescriptor(
                type=Parameter.Type.STRING,
                description=(
                    'The name of the frame to take the head\'s relative'
                    'position from.')))
        self.declare_parameter(
            'frequency', 60,
            ParameterDescriptor(
                type=Parameter.Type.INTEGER,
                description=('The polling rate (in hz).')))

    def _move_head(self):
        """Actuate motors when timer runs callback."""
        # get transform
        try:
            world_head_tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.head_frame,
                rclpy.time.Time())
        except TransformException as e:
            self.get_logger().info(
                f'Could not get {self.world_frame} -> {self.head_frame} transform: {e}',
                throttle_duration_sec=1)
            return

        quat = [
            world_head_tf.transform.rotation.x,
            world_head_tf.transform.rotation.y,
            world_head_tf.transform.rotation.z,
            world_head_tf.transform.rotation.w]
        _, pitch, yaw = Rotation.from_quat(quat).as_euler('xyz')
        # self.get_logger().info(f"{pitch:.3f} {yaw:.3f}")

        # NEW
        self.controller.set_pos(
            self.yaw_smoothener(yaw), self.pitch_smoothener(pitch))

    def safe_shutdown(self):
        """Return neck to safe position then deinit."""
        if self.shutting_down:
            return

        self.shutting_down = True
        self.get_logger().info('Returning neck to safe configuration')

        try:
            self.timer.cancel()

            self.controller.set_pos(0.0, -0.5)

            sleep(1)
        except Exception as e:
            self.get_logger().error(f'Shutdown failed: {e}', throttle_duration_sec=1)

def parse_motor_config(controller_config: dict) -> dict[MotorType, MotorConfig]:
    """Parse config json and create MotorConfigs for the yaw and pitch motors."""
    res: dict[MotorType, MotorConfig] = {}
    for motor_name in get_args(MotorType):
        motor_config: dict[str, int|float] = controller_config[motor_name]
        res[motor_name] = MotorConfig(
            id=motor_config['id'],
            center=motor_config['center'],
            range=(motor_config['min'], motor_config['max']),
            ratio=motor_config['ratio'],
            resolution=motor_config['resolution']
        )

    return res

def main(args=None):
    # get config file argument
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parsed_args, ros_args = parser.parse_known_args(args)

    with open(parsed_args.config, 'r') as f:
        config = json.load(f)

    rclpy.init(args=ros_args)

    motors_config = parse_motor_config(config)
    node = NeckNode(
        config['device_name'], config['baudrate'],
        motors_config)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_shutdown()
        node.destroy_node()