# Copyright (c) 2026 Dimas Abreu Archanjo Dutra
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
Launch test for fr3_motion's joint_impedance_controller.

Brings up franka_bringup/franka.launch.py with fake hardware and this package's
controllers.yaml (which registers joint_impedance_controller), loads and activates the
controller through the controller_manager services -- the scripted equivalent of
`ros2 control load_controller --set-state active joint_impedance_controller` -- drives it
with example_joint_impedance_commander, and verifies that nonzero effort values are
published on /joint_states.
"""

import unittest

from franka_bringup.testing.controller_service_client import ControllerServiceClient
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions
import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import JointState

CONTROLLER_NAME = 'joint_impedance_controller'


def generate_test_description():
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('fr3_motion'), 'config', 'controllers.yaml']
    )

    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py']
                )
            ]
        ),
        launch_arguments={
            'robot_type': 'fr3',
            'use_fake_hardware': 'true',
            'controllers_yaml': controllers_yaml,
        }.items(),
    )

    commander = Node(
        package='fr3_motion',
        executable='example_joint_impedance_commander',
        output='screen',
    )

    return LaunchDescription(
        [
            franka_launch,
            commander,
            TimerAction(period=3.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    )


class TestJointImpedanceController(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_joint_impedance_controller')

    def tearDown(self):
        self.node.destroy_node()

    def test_effort_published_after_activation(self):
        client = ControllerServiceClient(self.node)
        try:
            self.assertTrue(
                client.wait_for_services(timeout_sec=30.0),
                'controller_manager services never became available.',
            )
            self.assertTrue(client.load_controller(CONTROLLER_NAME))
            self.assertTrue(client.configure_controller(CONTROLLER_NAME))
            self.assertTrue(client.switch_controllers(activate=[CONTROLLER_NAME]))

            efforts = []
            sub = self.node.create_subscription(
                JointState, '/joint_states', lambda msg: efforts.append(list(msg.effort)), 10
            )

            deadline = self.node.get_clock().now() + Duration(seconds=15.0)
            nonzero_seen = False
            while self.node.get_clock().now() < deadline and not nonzero_seen:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if efforts and any(abs(e) > 1e-3 for e in efforts[-1]):
                    nonzero_seen = True
            self.node.destroy_subscription(sub)

            self.assertTrue(
                efforts,
                'No /joint_states messages received while joint_impedance_controller was '
                'active.',
            )
            self.assertTrue(
                nonzero_seen,
                'Effort values on /joint_states never became nonzero while '
                'joint_impedance_controller tracked a moving setpoint.',
            )
        finally:
            client.unload_controller(CONTROLLER_NAME)
            client.destroy()
