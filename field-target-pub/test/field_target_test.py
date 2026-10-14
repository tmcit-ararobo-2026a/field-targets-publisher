"""Exercise explicit profiles and mathematical bearings with synthetic world geometry."""

import json
import math
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from field_target_pub.msg import TargetSet


def test_fixed_target_profiles(tmp_path):
    """Require complete approved targets and refuse stale TF or wire angle guesses."""
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    executable = os.environ['FIELD_TARGET_EXECUTABLE']
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = Node('field_target_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    outputs = []
    node.create_subscription(TargetSet, '/field/targets', outputs.append, 10)
    transforms = TransformBroadcaster(node)

    def profile(team):
        """Create synthetic, explicitly approved fixture geometry for each fixed ID."""
        entries = []
        for identifier in range(7):
            entry = {'target_id': identifier, 'label': f'fixture-{identifier}',
                     'stop_pose': [float(identifier), 0.0, 0.0, 0.5]}
            if identifier:
                entry['aim_point'] = [2.0, 1.0, 2.0]
                entry['object_center'] = [float(identifier), 1.0, 0.5]
            entries.append(entry)
        return {'schema_version': 1, 'approved': True, 'team': team,
                'map_id': 'synthetic-map', 'map_frame': 'map', 'revision': 1,
                'targets': entries, 'static_objects': [
                    {'frame_id': 'desk_other_center', 'object_center': [4.0, 4.0, 0.5]}]}

    def run_case(config, expected_valid, expected_map='synthetic-map', yaw=0.0):
        """Run a profile and observe validity and TF-derived bearing."""
        path = tmp_path / 'targets.json'
        path.write_text(json.dumps(config))
        outputs.clear()
        process = subprocess.Popen([executable, '--ros-args', '-p', f'config_file:={path}',
                                    '-p', f'expected_map_id:={expected_map}',
                                    '-p', 'pose_timeout_s:=0.1'],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        try:
            end = time.monotonic() + 5
            while time.monotonic() < end:
                transform = TransformStamped()
                transform.header.stamp = node.get_clock().now().to_msg()
                transform.header.frame_id = 'map'
                transform.child_frame_id = 'base_link'
                transform.transform.rotation.z = math.sin(yaw / 2)
                transform.transform.rotation.w = math.cos(yaw / 2)
                transforms.sendTransform(transform)
                executor.spin_once(timeout_sec=0.01)
                if outputs and outputs[-1].valid == expected_valid:
                    if not expected_valid or outputs[-1].targets[1].bearing_valid:
                        break
            assert process.poll() is None
            assert outputs and outputs[-1].valid == expected_valid
            if expected_valid:
                current = outputs[-1]
                assert {target.target_id for target in current.targets} == set(range(7))
                assert all(target.stop_pose_valid for target in current.targets)
                assert not current.targets[0].aim_valid
                assert all(not target.angle_valid for target in current.targets)
                assert current.targets[1].stop_pose.pose.position.x == 1.0
                assert current.targets[1].aim_point.point.x == 2.0
                assert abs(current.targets[1].bearing_rad -
                           math.atan2(math.cos(yaw) - 2 * math.sin(yaw),
                                      2 * math.cos(yaw) + math.sin(yaw))) < 1e-5
                end = time.monotonic() + 0.2
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=0.01)
                assert not outputs[-1].targets[1].bearing_valid
            else:
                assert not outputs[-1].targets
        finally:
            process.terminate()
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=5)

    try:
        unapproved = profile('red')
        unapproved['approved'] = False
        run_case(unapproved, False)
        run_case(profile('red'), True)
        run_case(profile('blue'), True, yaw=math.pi / 2)
        run_case(profile('red'), False, expected_map='other-map')
        invalid_profiles = []
        missing = profile('red')
        missing['targets'].pop()
        invalid_profiles.append(missing)
        duplicate = profile('red')
        duplicate['targets'][1]['target_id'] = 0
        invalid_profiles.append(duplicate)
        malformed = profile('red')
        malformed['targets'][1]['stop_pose'] = [0, 0]
        invalid_profiles.append(malformed)
        for config in invalid_profiles:
            path = tmp_path / 'invalid.json'
            path.write_text(json.dumps(config))
            result = subprocess.run([executable, '--ros-args', '-p', f'config_file:={path}'],
                                    capture_output=True, timeout=5)
            assert result.returncode == 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
