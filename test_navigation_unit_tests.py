"""
Unit tests for sunnypilot navigation modules.
This addresses the critical issue of missing unit tests and verification.
"""
import unittest
import time
from unittest.mock import Mock, patch
import numpy as np

from sunnypilot.navd.helpers import Coordinate
from sunnypilot.navd.routing import BasicRouter, EnhancedRouteManager, RouteSegment
from sunnypilot.navd.navigation import PointToPointNavigation, NavInstructionController
from openpilot.selfdrive.common.metrics import get_all_metric_summaries, Metrics


class TestBasicRouter(unittest.TestCase):
    """Test the basic routing functionality."""
    
    def setUp(self):
        self.router = BasicRouter()
        
    def test_route_calculation(self):
        """Test that route calculation works correctly."""
        start = Coordinate(37.7749, -122.4194)  # San Francisco
        dest = Coordinate(37.7849, -122.4094)   # Nearby coordinate
        
        route = self.router.calculate_route(start, dest)
        
        self.assertIsNotNone(route)
        self.assertGreater(route.total_distance, 0)
        self.assertGreater(route.total_time, 0)
        self.assertGreater(len(route.segments), 0)
        
        # Verify all segments have valid data
        for segment in route.segments:
            self.assertIsInstance(segment, RouteSegment)
            self.assertGreaterEqual(segment.distance, 0)
            self.assertIsInstance(segment.maneuver_type, str)
    
    def test_short_route(self):
        """Test route calculation for very short distances."""
        start = Coordinate(37.7749, -122.4194)
        dest = Coordinate(37.7750, -122.4195)  # Very close
        
        route = self.router.calculate_route(start, dest)
        
        # For very short routes, it might return None (indicating arrival)
        if route is not None:
            self.assertLess(route.total_distance, 50)  # Less than 50m
    
    def test_same_start_destination(self):
        """Test when start and destination are the same."""
        coord = Coordinate(37.7749, -122.4194)
        
        route = self.router.calculate_route(coord, coord)
        
        # Should return None for same start/destination
        self.assertIsNone(route)


class TestEnhancedRouteManager(unittest.TestCase):
    """Test the enhanced route manager with real routing."""
    
    def setUp(self):
        self.route_manager = EnhancedRouteManager()
        
    def test_set_destination(self):
        """Test setting a destination with route calculation."""
        start_pos = Coordinate(37.7749, -122.4194)
        dest = Coordinate(37.7849, -122.4094)
        
        success = self.route_manager.set_destination(dest, start_pos)
        
        self.assertTrue(success)
        self.assertTrue(self.route_manager.active)
        self.assertIsNotNone(self.route_manager.current_route)
    
    def test_update_route(self):
        """Test updating route with current position."""
        start_pos = Coordinate(37.7749, -122.4194)
        dest = Coordinate(37.7849, -122.4094)
        
        success = self.route_manager.set_destination(dest, start_pos)
        self.assertTrue(success)
        
        # Update with a new position (slightly different from start)
        new_pos = Coordinate(37.7750, -122.4193)
        self.route_manager.update_route(new_pos)
        
        # Should still be active
        self.assertTrue(self.route_manager.active)
    
    def test_route_completion(self):
        """Test route completion detection."""
        start_pos = Coordinate(37.7749, -122.4194)
        dest = Coordinate(37.7849, -122.4094)
        
        success = self.route_manager.set_destination(dest, start_pos)
        self.assertTrue(success)
        
        # Check completion with position very close to destination
        close_pos = Coordinate(dest.latitude + 0.0001, dest.longitude + 0.0001)
        completed = self.route_manager.check_route_completion(close_pos, tolerance=50.0)
        
        self.assertTrue(completed)
        self.assertFalse(self.route_manager.active)
    
    def test_maneuver_info(self):
        """Test getting maneuver information."""
        start_pos = Coordinate(37.7749, -122.4194)
        dest = Coordinate(37.7849, -122.4094)
        
        success = self.route_manager.set_destination(dest, start_pos)
        self.assertTrue(success)
        
        maneuver_type, distance, angle = self.route_manager.get_maneuver_info(start_pos)
        
        self.assertIsInstance(maneuver_type, str)
        self.assertIsInstance(distance, (int, float))
        self.assertIsInstance(angle, (int, float))


class TestPointToPointNavigation(unittest.TestCase):
    """Test the main navigation system."""
    
    def setUp(self):
        # Mock the messaging system
        with patch('cereal.messaging.PubMaster'), \
             patch('cereal.messaging.SubMaster') as mock_submaster:
            
            # Mock a typical response for liveLocationKalman
            mock_msg = Mock()
            mock_msg.positionGeodetic.valid = True
            mock_msg.positionGeodetic.value = [37.7749, -122.4194, 0]  # lat, lon, alt
            mock_submaster.__getitem__.return_value = mock_msg
            mock_submaster.updated = {'liveLocationKalman': True}
            mock_submaster.update.return_value = None
            mock_submaster.frame = 0
            
            self.nav_system = PointToPointNavigation()
            # Manually set a current position for testing
            self.nav_system.current_position = Coordinate(37.7749, -122.4194)
    
    def test_set_destination(self):
        """Test setting a navigation destination."""
        # This should work since we set current_position manually
        result = self.nav_system.set_destination(37.7849, -122.4094)
        
        self.assertTrue(result)
        self.assertTrue(self.nav_system.nav_active)
        self.assertIsNotNone(self.nav_system.route_manager.current_route)
    
    def test_get_status(self):
        """Test getting navigation status."""
        # First set a destination
        self.nav_system.set_destination(37.7849, -122.4094)
        
        status = self.nav_system.get_status()
        
        self.assertIsInstance(status, dict)
        self.assertIn('active', status)
        self.assertIn('destination', status)
        self.assertIn('remaining_distance', status)
        self.assertIn('remaining_time', status)


class TestNavInstructionController(unittest.TestCase):
    """Test the navigation instruction controller."""
    
    def setUp(self):
        # Mock the messaging system
        with patch('cereal.messaging.SubMaster') as mock_submaster:
            mock_submaster.updated = {'navInstruction': False}
            mock_submaster.__getitem__.return_value = Mock()
            
            self.controller = NavInstructionController()
    
    def test_update(self):
        """Test updating the controller."""
        # This should work without errors
        self.controller.update(100)  # frame number
        
        # Verify no exceptions occurred
        self.assertIsNotNone(self.controller)


class TestMetricsIntegration(unittest.TestCase):
    """Test that metrics are properly recorded."""
    
    def test_metrics_recording(self):
        """Test that various metrics are being recorded."""
        from openpilot.selfdrive.common.metrics import record_metric, Metrics
        
        # Record some test metrics
        record_metric(Metrics.NAVIGATION_LATENCY_MS, 25.0, {"test": "navigation"})
        record_metric(Metrics.ROUTE_COMPLETION_RATE, 1.0, {"test": "route"})
        record_metric(Metrics.PLANNING_LATENCY_MS, 15.0, {"test": "planning"})
        
        # Get all metric summaries
        summaries = get_all_metric_summaries()
        
        # Verify that our metrics were recorded
        self.assertIn(Metrics.NAVIGATION_LATENCY_MS, summaries)
        self.assertIn(Metrics.ROUTE_COMPLETION_RATE, summaries)
        self.assertIn(Metrics.PLANNING_LATENCY_MS, summaries)
        
        # Verify the values are correct
        nav_summary = summaries[Metrics.NAVIGATION_LATENCY_MS]
        self.assertEqual(nav_summary.latest, 25.0)


if __name__ == '__main__':
    print("Running navigation unit tests...")
    
    # Create a test suite
    test_suite = unittest.TestSuite()
    
    # Add all test cases
    test_suite.addTest(unittest.makeSuite(TestBasicRouter))
    test_suite.addTest(unittest.makeSuite(TestEnhancedRouteManager))
    test_suite.addTest(unittest.makeSuite(TestPointToPointNavigation))
    test_suite.addTest(unittest.makeSuite(TestNavInstructionController))
    test_suite.addTest(unittest.makeSuite(TestMetricsIntegration))
    
    # Run the tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\nTests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%" if result.testsRun > 0 else "0%")
    
    if result.failures or result.errors:
        print("\nFailures/Errors:")
        for failure in result.failures + result.errors:
            print(failure[1])
    else:
        print("\nAll tests passed! ✓")