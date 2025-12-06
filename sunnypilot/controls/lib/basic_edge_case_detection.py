"""
Simplified Edge Case Detection for Sunnypilot2
Based on the advanced edge case detection concepts but optimized for hardware constraints.

This module implements basic edge case detection focused on the most critical scenarios
with minimal computational overhead.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class EdgeCaseType(Enum):
    """Types of edge cases that can be detected."""
    CONSTRUCTION_ZONE = "construction_zone"
    ADVERSE_WEATHER = "adverse_weather" 
    TRAFFIC_INCIDENT = "traffic_incident"
    STOPPED_TRAFFIC = "stopped_traffic"
    PEDESTRIAN_CROSSING = "pedestrian_crossing"


@dataclass
class EdgeCase:
    """Simple representation of an edge case."""
    case_type: EdgeCaseType
    confidence: float  # 0.0 to 1.0
    severity: int  # 1-5, 5 being most severe
    distance: Optional[float] = None  # Distance to case if applicable
    duration: Optional[float] = None  # Duration of case if applicable


@dataclass
class DetectionResult:
    """Result of edge case detection."""
    edge_cases: List[EdgeCase]
    safe_speed_multiplier: float  # Multiplier to apply to desired speed
    required_action: str  # Recommended action


class BasicEdgeCaseDetector:
    """
    Basic edge case detection optimized for Snapdragon 845 hardware constraints.
    
    Focuses on simple, rule-based detection of critical edge cases without
    complex machine learning components.
    """
    
    def __init__(self):
        """Initialize the basic edge case detector."""
        # Thresholds for various detections
        self.construction_speed_threshold = 15.0  # m/s below which construction likely
        self.lead_distance_threshold = 25.0       # m for stopped traffic detection
        self.lead_velocity_threshold = 1.0        # m/s for stopped traffic
        self.confidence_threshold = 0.6           # minimum confidence for reporting
        
    def detect_edge_cases(self, 
                         radar_data: Dict,
                         vision_data: Dict, 
                         car_state: Dict) -> DetectionResult:
        """
        Detect basic edge cases using radar, vision, and car state data.
        
        Args:
            radar_data: Dictionary containing radar information
            vision_data: Dictionary containing vision information  
            car_state: Dictionary containing car state information
            
        Returns:
            DetectionResult with edge cases and recommended actions
        """
        edge_cases = []
        
        # Detect stopped traffic (most common and critical case)
        stopped_traffic_case = self._detect_stopped_traffic(radar_data, car_state)
        if stopped_traffic_case:
            edge_cases.append(stopped_traffic_case)
        
        # Detect potential construction zones
        construction_case = self._detect_construction_zone(radar_data, car_state)
        if construction_case:
            edge_cases.append(construction_case)
        
        # Calculate safe speed multiplier and required action
        safe_speed_multiplier = self._calculate_safe_speed_multiplier(edge_cases)
        required_action = self._determine_required_action(edge_cases)
        
        return DetectionResult(
            edge_cases=edge_cases,
            safe_speed_multiplier=safe_speed_multiplier,
            required_action=required_action
        )
    
    def _detect_stopped_traffic(self, radar_data: Dict, car_state: Dict) -> Optional[EdgeCase]:
        """
        Detect stopped traffic ahead.
        
        Args:
            radar_data: Radar data from the car
            car_state: Current car state
            
        Returns:
            EdgeCase if stopped traffic detected, None otherwise
        """
        v_ego = car_state.get('vEgo', 0.0)
        
        # Check for lead vehicle with low relative velocity at close range
        if 'leadOne' in radar_data and radar_data['leadOne'].status:
            lead = radar_data['leadOne']
            if (lead.dRel < self.lead_distance_threshold and 
                abs(lead.vRel) < self.lead_velocity_threshold and
                v_ego > 5.0):  # Only relevant at moderate speeds
                # Calculate confidence based on distance and relative velocity
                confidence = max(0.6, min(1.0, 0.8 + (5.0 - lead.dRel) * 0.02))
                return EdgeCase(
                    case_type=EdgeCaseType.STOPPED_TRAFFIC,
                    confidence=confidence,
                    severity=4,
                    distance=lead.dRel
                )
        
        return None
    
    def _detect_construction_zone(self, radar_data: Dict, car_state: Dict) -> Optional[EdgeCase]:
        """
        Detect potential construction zone based on speed and lead vehicle behavior.
        
        Args:
            radar_data: Radar data from the car
            car_state: Current car state
            
        Returns:
            EdgeCase if construction zone detected, None otherwise
        """
        v_ego = car_state.get('vEgo', 0.0)
        
        # Construction zones often have reduced speeds
        if v_ego < self.construction_speed_threshold:
            # Check for multiple vehicles at similar low speeds
            slow_leads = 0
            for lead_id in ['leadOne', 'leadTwo']:
                if lead_id in radar_data and getattr(radar_data[lead_id], 'status', False):
                    lead = radar_data[lead_id]
                    if lead.vRel < 2.0 and lead.dRel < 50.0:  # Within 50m and slow
                        slow_leads += 1
            
            if slow_leads >= 1 and v_ego < 10.0:
                # High likelihood of construction or incident
                confidence = min(1.0, 0.5 + slow_leads * 0.2)
                return EdgeCase(
                    case_type=EdgeCaseType.CONSTRUCTION_ZONE,
                    confidence=confidence,
                    severity=3
                )
        
        return None
    
    def _calculate_safe_speed_multiplier(self, edge_cases: List[EdgeCase]) -> float:
        """
        Calculate speed multiplier based on detected edge cases.
        
        Args:
            edge_cases: List of detected edge cases
            
        Returns:
            Speed multiplier between 0.0 and 1.0 (1.0 = no change)
        """
        if not edge_cases:
            return 1.0
        
        # Determine the highest severity case
        max_severity = max(case.severity for case in edge_cases) if edge_cases else 1
        
        # Calculate multiplier based on severity
        if max_severity >= 4:
            return 0.6  # Significant reduction for high severity
        elif max_severity == 3:
            return 0.8  # Moderate reduction
        else:
            return 0.9  # Minor reduction
    
    def _determine_required_action(self, edge_cases: List[EdgeCase]) -> str:
        """
        Determine required action based on detected edge cases.
        
        Args:
            edge_cases: List of detected edge cases
            
        Returns:
            String describing the required action
        """
        if not edge_cases:
            return "CONTINUE_NORMAL"
        
        # Sort by severity to get the most critical case
        if edge_cases:
            critical_case = max(edge_cases, key=lambda x: x.severity)
            if critical_case.case_type == EdgeCaseType.STOPPED_TRAFFIC:
                return "PREPARE_TO_STOP"
            elif critical_case.case_type == EdgeCaseType.CONSTRUCTION_ZONE:
                return "REDUCE_SPEED_CAUTION"
            else:
                return "EXERCISE_CAUTION"
        
        return "CONTINUE_NORMAL"


def create_edge_case_detector() -> BasicEdgeCaseDetector:
    """
    Factory function to create an edge case detector optimized for 
    Snapdragon 845 hardware constraints.
    """
    return BasicEdgeCaseDetector()