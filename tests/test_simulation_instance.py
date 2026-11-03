#!/usr/bin/env python3
"""Regression for duplicate navigation after Gazebo has already exited."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1] / 'src/indoor_bringup/launch/simulation.launch.py'
spec = importlib.util.spec_from_file_location('indoor_simulation', SOURCE)
simulation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(simulation)


class InstanceGuardTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.proc = Path(self.directory.name)

    def tearDown(self):
        self.directory.cleanup()

    def process(self, args, domain='88', uri='http://127.0.0.1:11355', pid=10):
        path = self.proc / str(pid)
        path.mkdir()
        (path / 'cmdline').write_bytes(('\0'.join(args) + '\0').encode())
        (path / 'environ').write_bytes(f'ROS_DOMAIN_ID={domain}\0GAZEBO_MASTER_URI={uri}\0'.encode())

    def conflicts(self, own_pid=99):
        return simulation.instance_conflicts(self.proc, '88', 'http://localhost:11355', own_pid)

    def test_orphaned_navigation_without_gazebo_blocks_restart(self):
        self.process(['/ws/nav2_net_system/net_navigator_node', '--ros-args', '-r', '__ns:=/car1'])
        self.assertEqual(len(self.conflicts()), 1)

    def test_duplicate_launch_blocks_restart(self):
        self.process(['python3', '/opt/ros/humble/bin/ros2', 'launch', 'indoor_bringup', 'simulation.launch.py'])
        self.assertEqual(len(self.conflicts()), 1)

    def test_own_launch_allowed(self):
        self.process(['python3', 'ros2', 'launch', 'indoor_bringup', 'simulation.launch.py'])
        self.assertEqual(self.conflicts(own_pid=10), [])

    def test_other_domain_navigation_allowed(self):
        self.process(['/ws/net_navigator_node', '-r', '__ns:=/car1'], domain='186')
        self.assertEqual(self.conflicts(), [])

    def test_other_namespace_allowed(self):
        self.process(['/ws/net_navigator_node', '-r', '__ns:=/car2'])
        self.assertEqual(self.conflicts(), [])

    def test_same_master_blocks_even_in_other_ros_domain(self):
        self.process(['gzserver', '/map.world'], domain='186')
        self.assertEqual(len(self.conflicts()), 1)

    def test_other_master_allowed(self):
        self.process(['gzserver', '/map.world'], uri='http://localhost:11356')
        self.assertEqual(self.conflicts(), [])


if __name__ == '__main__':
    unittest.main()
