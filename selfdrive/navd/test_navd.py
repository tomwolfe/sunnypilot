"""
Unit tests for Navigate on Autopilot implementation
Copyright (c) 2025, sunnypilot community
"""
import sys
import os
# Add the parent directory to path to import from selfdrive
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from unittest.mock import Mock
from openpilot.sunnypilot.navd.helpers import Coordinate, distance_along_geometry
from openpilot.selfdrive.navd.safety import NavSafetyManager
# Import RouteEngine from the main navd module (selfdrive/navd.py)
from openpilot.selfdrive.navd import RouteEngine


class TestCoordinate:
    def test_coordinate_creation(self):
        """Test coordinate creation and string representation"""
        coord = Coordinate(40.7128, -74.0060)  # New York coordinates
        assert coord.latitude == 40.7128
        assert coord.longitude == -74.0060

    def test_coordinate_distance(self):
        """Test distance calculation between coordinates"""
        coord1 = Coordinate(40.7128, -74.0060)  # New York
        coord2 = Coordinate(40.7589, -73.9851)  # Times Square, NY (close)

        distance = coord1.distance_to(coord2)
        # Should be a reasonable distance (less than 10km)
        assert distance < 10000
        assert distance > 0

    def test_coordinate_equality(self):
        """Test coordinate equality"""
        coord1 = Coordinate(40.7128, -74.0060)
        coord2 = Coordinate(40.7128, -74.0060)
        coord3 = Coordinate(41.0, -74.0)

        assert coord1 == coord2
        assert coord1 != coord3


class TestRouteEngine:
    def setup_method(self):
        """Set up test route engine"""
        self.route_engine = RouteEngine()

        # Create test route with 3 coordinates
        self.test_coordinates = [
            Coordinate(40.7128, -74.0060),
            Coordinate(40.7130, -74.0050),
            Coordinate(40.7140, -74.0040)
        ]
    
    def test_route_update(self):
        """Test updating route with coordinates"""
        self.route_engine.update_route(self.test_coordinates)

        assert self.route_engine.route_valid
        assert len(self.route_engine.coordinates) == 3

    def test_position_update(self):
        """Test updating position along route"""
        self.route_engine.update_route(self.test_coordinates)

        # Position close to first coordinate
        current_pos = Coordinate(40.7129, -74.0059)
        self.route_engine.update_position(current_pos)

        # Should have a valid route progress
        assert self.route_engine.route_progress >= 0

    def test_distance_calculation(self):
        """Test distance along geometry calculation"""
        # Use the helper function directly
        current_pos = Coordinate(40.7129, -74.0059)
        distance = distance_along_geometry(self.test_coordinates, current_pos)

        assert distance >= 0


class TestNavSafetyManager:
    def setup_method(self):
        """Set up test safety manager"""
        self.safety_manager = NavSafetyManager()

        # Create mock SubMaster
        self.sm = Mock()
        self.sm.updated = {'navInstruction': True}
        self.sm['carState'] = Mock()
        self.sm['carState'].vEgo = 15.0  # 15 m/s
        self.sm.get = Mock()
        self.sm.get.return_value = Mock()
        
    def test_speed_compliance(self):
        """Test speed compliance checks"""
        # Test compliance with appropriate speed
        compliant = self.safety_manager.check_speed_compliance(10.0, speed_limit=15.0)
        assert compliant

        # Test non-compliance with excessive speed
        non_compliant = self.safety_manager.check_speed_compliance(20.0, speed_limit=10.0)
        assert not non_compliant

        # Test with turn speed limit
        turn_compliant = self.safety_manager.check_speed_compliance(8.0, turn_speed_limit=10.0)
        assert turn_compliant

        turn_non_compliant = self.safety_manager.check_speed_compliance(15.0, turn_speed_limit=10.0)
        assert not turn_non_compliant

    def test_route_deviation(self):
        """Test route deviation detection"""
        current_pos = Coordinate(40.7128, -74.0060)
        route_pos = Coordinate(40.7129, -74.0061)  # Close route position

        # Within acceptable range
        on_route = self.safety_manager.update_position(current_pos, route_pos)
        assert on_route

        # Far from route
        far_route_pos = Coordinate(41.0, -75.0)  # Very far
        for _ in range(6):  # Trigger multiple violations
            off_route = self.safety_manager.update_position(current_pos, far_route_pos)
            if not off_route:
                break
        assert not off_route

    def test_route_validity(self):
        """Test route validity checks"""
        # Initially should be valid
        is_valid = self.safety_manager.check_route_validity(self.sm)
        assert is_valid

        # Simulate timeout by setting last valid time far in past
        self.safety_manager.last_valid_nav_time = 0
        is_valid_after_timeout = self.safety_manager.check_route_validity(self.sm)
        assert not is_valid_after_timeout


