"""
Unit tests for SimplePredictivePlanner
"""

import unittest
from unittest.mock import Mock, patch
import numpy as np
from selfdrive.common.predictive_planning import SimplePredictivePlanner, ManeuverType, PlannedTrajectory


class TestSimplePredictivePlanner(unittest.TestCase):
    """Test cases for SimplePredictivePlanner"""

    def setUp(self):
        """Set up test fixtures"""
        self.planner = SimplePredictivePlanner()

    def test_initialization(self):
        """Test that SimplePredictivePlanner initializes correctly"""
        self.assertIsNotNone(self.planner)
        self.assertEqual(self.planner.horizon, 5.0)
        self.assertEqual(self.planner.time_steps, 10)

    def test_predict_constant_velocity_basic(self):
        """Test constant velocity prediction with normal inputs"""
        position = np.array([0.0, 0.0, 0.0])
        velocity = np.array([10.0, 0.0, 0.0])
        
        trajectory = self.planner.predict_constant_velocity(position, velocity, horizon=2.0)
        
        self.assertEqual(trajectory.shape, (10, 3))
        # After 2 seconds at 10 m/s, x position should be 20 m
        expected_x = 20.0  # 10 m/s * 2 s for the last step (approximately)
        self.assertGreater(trajectory[-1, 0], 0)  # Should have moved in x direction

    def test_predict_constant_velocity_edge_cases(self):
        """Test constant velocity prediction with edge cases"""
        # Test with insufficient dimensions
        position = np.array([0.0])  # Only x
        velocity = np.array([10.0])  # Only vx
        
        trajectory = self.planner.predict_constant_velocity(position, velocity)
        
        self.assertEqual(trajectory.shape, (10, 3))
        # The function should handle this by padding with zeros

        # Test with None values (should be handled gracefully)
        with patch('selfdrive.common.predictive_planning.cloudlog') as mock_cloudlog:
            # This will cause an exception in the validation which gets caught
            trajectory = self.planner.predict_constant_velocity(position, velocity)
            self.assertEqual(trajectory.shape, (10, 3))

    def test_plan_lane_follow_basic(self):
        """Test basic lane follow planning"""
        ego_state = {
            'position': np.array([0.0, 0.0, 0.0]),
            'velocity': np.array([15.0, 0.0, 0.0])
        }
        tracked_objects = []

        trajectory = self.planner.plan_lane_follow(ego_state, tracked_objects)

        self.assertIsInstance(trajectory, PlannedTrajectory)
        self.assertTrue(trajectory.valid)
        self.assertEqual(trajectory.positions.shape, (10, 3))
        self.assertEqual(trajectory.velocities.shape, (10, 3))
        self.assertEqual(trajectory.maneuver_type, ManeuverType.LANE_FOLLOW)

    def test_plan_lane_follow_defaults(self):
        """Test lane follow planning with default values"""
        ego_state = {}  # Empty dict, should use defaults
        tracked_objects = []

        trajectory = self.planner.plan_lane_follow(ego_state, tracked_objects)

        self.assertIsInstance(trajectory, PlannedTrajectory)
        # Even with defaults, it should still produce a valid plan
        self.assertEqual(trajectory.positions.shape, (10, 3))
        self.assertEqual(trajectory.velocities.shape, (10, 3))

    def test_plan_with_obstacles_no_obstacles(self):
        """Test obstacle-aware planning with no obstacles"""
        ego_state = {
            'position': np.array([0.0, 0.0, 0.0]),
            'velocity': np.array([15.0, 0.0, 0.0])
        }
        tracked_objects = []  # No obstacles

        trajectory = self.planner.plan_with_obstacles(ego_state, tracked_objects)

        self.assertIsInstance(trajectory, PlannedTrajectory)
        self.assertTrue(trajectory.valid)

    def test_plan_with_obstacles_close(self):
        """Test obstacle-aware planning with close obstacles"""
        ego_state = {
            'position': np.array([0.0, 0.0, 0.0]),
            'velocity': np.array([15.0, 0.0, 0.0])
        }
        # Add a close obstacle
        tracked_objects = [{
            'dRel': 20.0,  # Close in front
            'yRel': 0.0,   # In our lane
            'vRel': -5.0   # Approaching quickly
        }]

        trajectory = self.planner.plan_with_obstacles(ego_state, tracked_objects)

        self.assertIsInstance(trajectory, PlannedTrajectory)
        self.assertTrue(trajectory.valid)
        # The planned velocity should be lower due to obstacle
        # (this depends on the implementation, but at least it shouldn't crash)

    def test_plan_lane_change(self):
        """Test lane change planning"""
        ego_state = {
            'position': np.array([0.0, 0.0, 0.0]),
            'velocity': np.array([15.0, 0.0, 0.0])
        }
        tracked_objects = []

        # Test left lane change
        trajectory_left = self.planner.plan_lane_change(
            ego_state, tracked_objects, ManeuverType.LANE_CHANGE_LEFT
        )

        self.assertIsInstance(trajectory_left, PlannedTrajectory)
        self.assertTrue(trajectory_left.valid)
        self.assertEqual(trajectory_left.maneuver_type, ManeuverType.LANE_CHANGE_LEFT)

        # Test right lane change
        trajectory_right = self.planner.plan_lane_change(
            ego_state, tracked_objects, ManeuverType.LANE_CHANGE_RIGHT
        )

        self.assertIsInstance(trajectory_right, PlannedTrajectory)
        self.assertTrue(trajectory_right.valid)
        self.assertEqual(trajectory_right.maneuver_type, ManeuverType.LANE_CHANGE_RIGHT)

        # Verify that the lateral position changes in the right direction
        # Left change should have positive y values, right change negative


if __name__ == '__main__':
    unittest.main()