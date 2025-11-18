"""
Configuration settings for the Sunnypilot Validation System.
"""

# General
SLEEP_TIME_PERCEPTION = 2.0  # Time to wait for perception data in seconds
PERCEPTION_TEST_FRAMES = 10  # Number of frames to process for perception performance test
PERCEPTION_FPS_TOLERANCE = 0.8  # Allow 20% tolerance for FPS target

# Navigation System
NAV_ORIGIN = (40.7128, -74.0060)  # NYC coordinates
NAV_DESTINATION = (40.7589, -73.9851)  # Times Square coordinates
NAV_PLANNING_TIMEOUT = 10  # Timeout for navigation route planning in seconds

# Performance Targets (for comma three hardware)
PERCEPTION_TARGET_FPS = 20  # Target 20 FPS for real-time performance
MEMORY_TARGET_MB = 1433.6  # 1.4GB in MB

# Point to Point Driving
DESTINATION_LATITUDE = 40.7589
DESTINATION_LONGITUDE = -73.9851
DESTINATION_NAME = "Times Square"
DESTINATION_ARRIVAL_RADIUS = 10.0

# Workflow Assessment Criteria
ASSESSMENT_CATEGORIES_SCORE_THRESHOLD = 85  # Percentage
ASSESSMENT_SAFETY_RESOLVED_REQUIRED = True
ASSESSMENT_OVERALL_GRADE_THRESHOLD = 95  # Percentage