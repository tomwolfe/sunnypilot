import unittest
import numpy as np
from unittest.mock import Mock

from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, ACCEL_MIN, ACCEL_MAX

class TestLongitudinalPlanner(unittest.TestCase):

  def setUp(self):
    # Mock CP (CarParams) and CP_SP (CarParams_SunnyPilot)
    self.CP = Mock()
    self.CP.mass = 1500
    self.CP.wheelbase = 2.7
    self.CP.steerRatio = 15
    self.CP_SP = Mock() # sunnypilot specific car params if needed

    self.planner = LongitudinalPlanner(self.CP, self.CP_SP)

  def test_get_model_confidence_values(self):
    sm = {
        'modelV2': Mock()
    }
    sm['modelV2'].meta = Mock()

    # Test case 1: pathReliability exists and is valid
    sm['modelV2'].meta.pathReliability = 0.8
    path_reliability = self.planner._get_model_confidence_values(sm)
    self.assertEqual(path_reliability, 0.8)

    # Test case 2: pathReliability exists but is None (should default to 1.0)
    sm['modelV2'].meta.pathReliability = None
    path_reliability = self.planner._get_model_confidence_values(sm)
    self.assertEqual(path_reliability, 1.0)

    # Test case 3: pathReliability does not exist
    del sm['modelV2'].meta.pathReliability
    path_reliability = self.planner._get_model_confidence_values(sm)
    self.assertEqual(path_reliability, 1.0)

  def test_adjust_accel_clip_for_confidence_and_reliability(self):
    base_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    base_accel_rate_limit = 0.05

    # Test case 1: High path reliability
    path_reliability = 1.0
    accel_clip, accel_rate_limit = self.planner._adjust_accel_clip_for_confidence_and_reliability(
        list(base_accel_clip), base_accel_rate_limit, path_reliability)
    np.testing.assert_allclose(accel_clip, base_accel_clip)
    self.assertEqual(accel_rate_limit, base_accel_rate_limit)

    # Test case 2: Moderate path reliability (< 0.7)
    path_reliability = 0.6
    accel_clip, accel_rate_limit = self.planner._adjust_accel_clip_for_confidence_and_reliability(
        list(base_accel_clip), base_accel_rate_limit, path_reliability)
    expected_accel_max = ACCEL_MAX * 0.85
    expected_accel_min = ACCEL_MIN * 0.90 # Note: ACCEL_MIN is negative
    self.assertLessEqual(accel_clip[1], expected_accel_max + 1e-9) # Allow for floating point inaccuracies
    self.assertGreaterEqual(accel_clip[0], expected_accel_min - 1e-9)
    self.assertEqual(accel_rate_limit, base_accel_rate_limit)

    # Test case 3: Low path reliability (< 0.5)
    path_reliability = 0.4
    accel_clip, accel_rate_limit = self.planner._adjust_accel_clip_for_confidence_and_reliability(
        list(base_accel_clip), base_accel_rate_limit, path_reliability)
    expected_accel_max_moderate = ACCEL_MAX * 0.85 # from first if
    expected_accel_max_low = ACCEL_MAX * 0.7 # from second if
    expected_accel_min_moderate = ACCEL_MIN * 0.90
    expected_accel_min_low = ACCEL_MIN * 0.8 # Note: ACCEL_MIN is negative
    self.assertLessEqual(accel_clip[1], min(expected_accel_max_moderate, expected_accel_max_low) + 1e-9)
    self.assertGreaterEqual(accel_clip[0], max(expected_accel_min_moderate, expected_accel_min_low) - 1e-9)
    self.assertEqual(accel_rate_limit, 0.03) # Should be reduced

  def test_calculate_safety_factor(self):
    sm = Mock()

    # Test case 1: High overall safety score, high path reliability
    overall_safety_score = 1.0
    path_reliability = 1.0
    safety_factor = self.planner._calculate_safety_factor(sm, overall_safety_score, path_reliability)
    self.assertEqual(safety_factor, 1.0)

    # Test case 2: Moderate overall safety score (0.5), high path reliability
    overall_safety_score = 0.5
    path_reliability = 1.0
    safety_factor = self.planner._calculate_safety_factor(sm, overall_safety_score, path_reliability)
    self.assertEqual(safety_factor, 0.8)

    # Test case 3: Low overall safety score (0.3), high path reliability
    overall_safety_score = 0.3
    path_reliability = 1.0
    safety_factor = self.planner._calculate_safety_factor(sm, overall_safety_score, path_reliability)
    self.assertEqual(safety_factor, 0.65) # min(0.65, 0.7) from path_reliability check

    # Test case 4: Critical overall safety score (0.1), high path reliability
    overall_safety_score = 0.1
    path_reliability = 1.0
    safety_factor = self.planner._calculate_safety_factor(sm, overall_safety_score, path_reliability)
    self.assertEqual(safety_factor, 0.5)

    # Test case 5: High overall safety score, low path reliability (< 0.7)
    overall_safety_score = 1.0
    path_reliability = 0.6
    safety_factor = self.planner._calculate_safety_factor(sm, overall_safety_score, path_reliability)
    self.assertEqual(safety_factor, 0.7)

    # Test case 6: High overall safety score, very low path reliability (< 0.5)
    overall_safety_score = 1.0
    path_reliability = 0.4
    safety_factor = self.planner._calculate_safety_factor(sm, overall_safety_score, path_reliability)
    self.assertEqual(safety_factor, 0.5)

    # Test case 7: Moderate overall safety score (0.5), low path reliability (0.6)
    overall_safety_score = 0.5
    path_reliability = 0.6
    safety_factor = self.planner._calculate_safety_factor(sm, overall_safety_score, path_reliability)
    self.assertEqual(safety_factor, 0.7) # min(0.8, 0.7)

    # Test case 8: Critical overall safety score (0.1), very low path reliability (0.4)
    overall_safety_score = 0.1
    path_reliability = 0.4
    safety_factor = self.planner._calculate_safety_factor(sm, overall_safety_score, path_reliability)
    self.assertEqual(safety_factor, 0.5) # min(0.5, 0.5)

if __name__ == '__main__':
  unittest.main()
