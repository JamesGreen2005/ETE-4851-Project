"""Regression for the read-only rclpy Node.handle property seen on the Pi."""
import ast
from pathlib import Path
import unittest


class RosNameTests(unittest.TestCase):
    def test_explorer_does_not_overwrite_ros_node_handle(self):
        source=Path(__file__).parents[1]/'robot_explorer/explore.py'
        tree=ast.parse(source.read_text())
        attributes=[n for n in ast.walk(tree) if isinstance(n,ast.Attribute)
                    and isinstance(n.value,ast.Name) and n.value.id=='self']
        self.assertFalse(any(n.attr=='handle' for n in attributes))
        self.assertTrue(any(n.attr=='goal_handle' for n in attributes))
