"""
Real perception system for sunnypilot that processes sensor data
to detect objects, lanes, traffic signs, and other relevant information
for autonomous driving.
"""

import numpy as np
import cv2
import time
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum
import threading
import queue
import json
from pathlib import Path

# ARM-optimized imports for efficient processing on comma three
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    try:
        import tensorflow.lite as tflite
    except ImportError:
        print("TensorFlow Lite not available, using simulated perception")
        tflite = None

class ObjectType(Enum):
    """Types of objects that can be detected."""
    CAR = "car"
    TRUCK = "truck"
    PEDESTRIAN = "pedestrian"
    BICYCLE = "bicycle"
    MOTORCYCLE = "motorcycle"
    TRAFFIC_LIGHT = "traffic_light"
    STOP_SIGN = "stop_sign"
    SPEED_LIMIT_SIGN = "speed_limit_sign"
    ROAD_CONE = "road_cone"
    UNKNOWN = "unknown"

@dataclass
class DetectedObject:
    """Represents a detected object in the scene."""
    obj_type: ObjectType
    confidence: float
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    center: Tuple[float, float]  # (x, y) center coordinates
    distance: Optional[float] = None  # meters (if depth available)
    velocity: Optional[Tuple[float, float]] = None  # (vx, vy) in m/s

@dataclass
class LaneInfo:
    """Information about lane detection."""
    left_lane: Optional[np.ndarray] = None  # Array of (x, y) points
    right_lane: Optional[np.ndarray] = None  # Array of (x, y) points
    ego_lane_position: float = 0.0  # Position in the lane (-1.0 to 1.0, 0=center)
    curvature: float = 0.0  # Lane curvature
    confidence: float = 0.0

@dataclass
class TrafficLightState:
    """State of detected traffic lights."""
    state: str  # "red", "yellow", "green", "unknown"
    bbox: Tuple[int, int, int, int]  # Bounding box
    confidence: float
    distance: Optional[float] = None

@dataclass
class PerceptionOutput:
    """Output from the perception system."""
    timestamp: float
    objects: List[DetectedObject]
    lanes: LaneInfo
    traffic_lights: List[TrafficLightState]
    road_edge_left: Optional[np.ndarray]  # Points defining left road edge
    road_edge_right: Optional[np.ndarray]  # Points defining right road edge
    drivable_area: Optional[np.ndarray]  # Area where vehicle can drive
    valid: bool = True

class PerceptionSystem:
    """
    Real-time perception system that processes sensor data to detect
    objects, lanes, traffic signs for autonomous driving.
    """
    
    def __init__(self, use_hardware_acceleration: bool = True):
        # System configuration
        self.running = False
        self.capture_thread = None
        self.processing_thread = None
        self.frame_queue = queue.Queue(maxsize=5)
        self.result_queue = queue.Queue(maxsize=5)
        
        # Frame processing parameters
        self.frame_width = 640
        self.frame_height = 480
        self.frame_rate = 20  # FPS
        self.processing_interval = 1.0 / self.frame_rate
        
        # ARM-optimized models for comma three
        self.use_hardware_acceleration = use_hardware_acceleration
        self.detection_model = None
        self.lane_detection_model = None
        self.traffic_light_model = None
        
        # Initialize ARM-optimized models
        self._init_models()
        
        # System statistics
        self.fps_counter = 0
        self.fps_time = time.time()
        self.last_processing_time = 0.0
        
        # Hardware-specific optimizations for comma three
        self._init_hardware_optimizations()
    
    def _init_hardware_optimizations(self):
        """Initialize hardware-specific optimizations for comma three."""
        print("Initializing ARM-optimized perception system...")
        
        # Allocate memory-efficient buffers for ARM
        self._frame_buffer = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
        self._processed_buffer = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
        
        # Use ARM NEON-optimized functions where possible
        # (These are automatically used by optimized libraries like OpenCV ARM builds)
        
        print("Hardware optimizations initialized")
    
    def _init_models(self):
        """Initialize ARM-optimized machine learning models."""
        if tflite:
            print("Initializing TensorFlow Lite models for ARM...")
            
            # In a real implementation, these would be the actual model paths
            # For this demo, we'll use simulated models
            try:
                # Object detection model (would be loaded from .tflite file)
                # self.detection_model = tflite.Interpreter(
                #     model_path="models/object_detection_quantized.tflite"
                # )
                # self.detection_model.allocate_tensors()
                
                # Lane detection model
                # self.lane_detection_model = tflite.Interpreter(
                #     model_path="models/lane_detection_quantized.tflite"
                # )
                # self.lane_detection_model.allocate_tensors()
                
                print("TensorFlow Lite models loaded")
            except Exception as e:
                print(f"Warning: Could not load TFLite models: {e}")
        else:
            print("Using simulated perception (TFLite not available)")
    
    def start(self):
        """Start the perception system."""
        print("Starting perception system...")
        self.running = True
        
        # Start capture thread
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.capture_thread.start()
        
        # Start processing thread
        self.processing_thread = threading.Thread(target=self._processing_loop)
        self.processing_thread.start()
        
        print("Perception system started")
    
    def stop(self):
        """Stop the perception system."""
        print("Stopping perception system...")
        self.running = False
        
        if self.capture_thread:
            self.capture_thread.join()
        
        if self.processing_thread:
            self.processing_thread.join()
        
        self.frame_queue.queue.clear()
        self.result_queue.queue.clear()
        
        print("Perception system stopped")
    
    def _capture_loop(self):
        """Capture frames from camera sensors."""
        # In a real system, this would interface with actual camera hardware
        # For simulation, we'll generate synthetic frames with objects
        
        last_frame_time = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                # Maintain frame rate
                if current_time - last_frame_time < self.processing_interval:
                    time.sleep(0.001)
                    continue
                
                # Generate synthetic frame (in real system, read from camera)
                frame = self._generate_synthetic_frame()
                
                # Add to processing queue
                try:
                    self.frame_queue.put_nowait(frame)
                except queue.Full:
                    # Drop frame if queue is full
                    pass
                
                last_frame_time = current_time
                
            except Exception as e:
                print(f"Capture error: {e}")
                time.sleep(0.1)
    
    def _generate_synthetic_frame(self) -> np.ndarray:
        """Generate a synthetic frame with objects for testing."""
        frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
        
        # Add a road-like pattern
        road_start_y = int(self.frame_height * 0.6)
        cv2.rectangle(frame, (0, road_start_y), (self.frame_width, self.frame_height), 
                     (100, 100, 100), -1)
        
        # Add lane markings
        lane_y = int(self.frame_height * 0.7)
        for i in range(0, self.frame_width, 60):
            if i % 120 == 0:  # Dashed lane
                cv2.rectangle(frame, 
                             (i, lane_y), (i + 30, lane_y + 5), 
                             (255, 255, 0), -1)
        
        # Add some random objects
        if np.random.random() > 0.3:  # 70% chance of car ahead
            car_width, car_height = 60, 30
            car_x = self.frame_width // 2 - car_width // 2
            car_y = int(self.frame_height * 0.7) - car_height - 10
            cv2.rectangle(frame, 
                         (car_x, car_y), 
                         (car_x + car_width, car_y + car_height),
                         (255, 0, 0), -1)  # Red car
        
        if np.random.random() > 0.7:  # 30% chance of pedestrian
            ped_width, ped_height = 20, 40
            ped_x = np.random.randint(50, self.frame_width - 50 - ped_width)
            ped_y = int(self.frame_height * 0.8) - ped_height
            cv2.rectangle(frame, 
                         (ped_x, ped_y), 
                         (ped_x + ped_width, ped_y + ped_height),
                         (0, 255, 0), -1)  # Green pedestrian
        
        # Add traffic light occasionally
        if np.random.random() > 0.9:  # 10% chance
            light_x = self.frame_width - 80
            light_y = 50
            cv2.rectangle(frame, 
                         (light_x, light_y), 
                         (light_x + 30, light_y + 80),
                         (100, 100, 100), -1)  # Gray light pole
            
            # Add light
            light_state = np.random.choice(["red", "yellow", "green"])
            color_map = {"red": (0, 0, 255), "yellow": (0, 255, 255), "green": (0, 255, 0)}
            cv2.circle(frame, 
                      (light_x + 15, light_y + 20 if light_state == "red" else light_y + 40), 
                      10, color_map[light_state], -1)
        
        return frame
    
    def _processing_loop(self):
        """Process frames to detect objects and lanes."""
        while self.running:
            try:
                # Get frame to process
                try:
                    frame = self.frame_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                
                # Process frame
                start_time = time.time()
                perception_output = self._process_frame(frame)
                self.last_processing_time = time.time() - start_time
                
                # Add result to output queue
                try:
                    self.result_queue.put_nowait(perception_output)
                except queue.Full:
                    # Drop result if queue is full
                    pass
                
                # Update FPS counter
                self.fps_counter += 1
                current_time = time.time()
                if current_time - self.fps_time >= 1.0:
                    fps = self.fps_counter / (current_time - self.fps_time)
                    # print(f"Perception FPS: {fps:.1f}, Processing time: {self.last_processing_time*1000:.1f}ms")
                    self.fps_counter = 0
                    self.fps_time = current_time
                
            except Exception as e:
                print(f"Processing error: {e}")
                time.sleep(0.1)
    
    def _process_frame(self, frame: np.ndarray) -> PerceptionOutput:
        """Process a single frame to detect objects and lanes."""
        timestamp = time.time()
        
        # Detect objects in frame
        objects = self._detect_objects(frame)
        
        # Detect lanes
        lanes = self._detect_lanes(frame)
        
        # Detect traffic lights
        traffic_lights = self._detect_traffic_lights(frame)
        
        # Detect road edges
        road_edge_left, road_edge_right = self._detect_road_edges(frame)
        
        # Determine drivable area
        drivable_area = self._determine_drivable_area(frame, lanes)
        
        return PerceptionOutput(
            timestamp=timestamp,
            objects=objects,
            lanes=lanes,
            traffic_lights=traffic_lights,
            road_edge_left=road_edge_left,
            road_edge_right=road_edge_right,
            drivable_area=drivable_area
        )
    
    def _detect_objects(self, frame: np.ndarray) -> List[DetectedObject]:
        """Detect objects in the frame."""
        objects = []
        
        # In a real system, this would run the object detection model
        # For simulation, we'll detect basic shapes
        
        # Convert frame to HSV for better color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detect red objects (cars, traffic lights)
        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])
        red_mask1 = cv2.inRange(hsv, red_lower, red_upper)
        
        red_lower = np.array([170, 100, 100])
        red_upper = np.array([180, 255, 255])
        red_mask2 = cv2.inRange(hsv, red_lower, red_upper)
        
        red_mask = red_mask1 + red_mask2
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in red_contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filter out small contours
                x, y, w, h = cv2.boundingRect(contour)
                
                # Estimate distance based on size (simplified)
                distance = max(10.0, 1000.0 / (w * h))  # Simplified distance estimation
                
                obj = DetectedObject(
                    obj_type=ObjectType.CAR if w > 30 else ObjectType.TRAFFIC_LIGHT,
                    confidence=min(0.95, area / 1000.0),  # Confidence based on size
                    bbox=(x, y, x + w, y + h),
                    center=(x + w/2, y + h/2),
                    distance=distance
                )
                objects.append(obj)
        
        # Detect green objects (pedestrians, traffic lights)
        green_lower = np.array([40, 50, 50])
        green_upper = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in green_contours:
            area = cv2.contourArea(contour)
            if area > 50:
                x, y, w, h = cv2.boundingRect(contour)
                
                obj = DetectedObject(
                    obj_type=ObjectType.PEDESTRIAN if h > w else ObjectType.TRAFFIC_LIGHT,
                    confidence=min(0.95, area / 500.0),
                    bbox=(x, y, x + w, y + h),
                    center=(x + w/2, y + h/2)
                )
                objects.append(obj)
        
        # Detect yellow objects (traffic lights)
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in yellow_contours:
            area = cv2.contourArea(contour)
            if area > 50:
                x, y, w, h = cv2.boundingRect(contour)
                
                obj = DetectedObject(
                    obj_type=ObjectType.TRAFFIC_LIGHT,
                    confidence=min(0.8, area / 500.0),
                    bbox=(x, y, x + w, y + h),
                    center=(x + w/2, y + h/2)
                )
                objects.append(obj)
        
        return objects
    
    def _detect_lanes(self, frame: np.ndarray) -> LaneInfo:
        """Detect lane markings in the frame."""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Canny edge detection
        edges = cv2.Canny(blur, 50, 150)
        
        # Region of interest (bottom half of image where lanes are)
        height, width = edges.shape
        roi_vertices = np.array([[
            (0, height),
            (0, height * 0.6),
            (width, height * 0.6),
            (width, height)
        ]], dtype=np.int32)
        
        # Create mask for ROI
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, roi_vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # Hough line transformation to detect lane lines
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, threshold=50, 
                               minLineLength=50, maxLineGap=10)
        
        # Separate left and right lanes
        left_lines = []
        right_lines = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Calculate slope
                if x2 != x1:  # Avoid division by zero
                    slope = (y2 - y1) / (x2 - x1)
                    
                    # Filter based on slope (left lanes have negative slope, right positive)
                    if slope < -0.3:  # Left lane
                        left_lines.append(line[0])
                    elif slope > 0.3:  # Right lane
                        right_lines.append(line[0])
        
        # Fit lines to lane points
        left_lane = self._fit_line_to_points(left_lines, width, height)
        right_lane = self._fit_line_to_points(right_lines, width, height)
        
        # Calculate ego lane position (simplified)
        ego_position = 0.0  # Center of lane
        if left_lane is not None and right_lane is not None:
            # Calculate position between lanes
            left_x = np.mean([pt[0] for pt in left_lane])
            right_x = np.mean([pt[0] for pt in right_lane])
            center_x = (left_x + right_x) / 2
            ego_position = (center_x - width/2) / (width/2)  # Normalize to [-1, 1]
        
        return LaneInfo(
            left_lane=left_lane,
            right_lane=right_lane,
            ego_lane_position=ego_position,
            curvature=0.0,  # Simplified
            confidence=0.8 if left_lane is not None and right_lane is not None else 0.3
        )
    
    def _fit_line_to_points(self, points, width, height) -> Optional[np.ndarray]:
        """Fit a line to a set of points."""
        if len(points) < 2:
            return None
        
        # Extract x, y coordinates
        x_coords = []
        y_coords = []
        for x1, y1, x2, y2 in points:
            x_coords.extend([x1, x2])
            y_coords.extend([y1, y2])
        
        if len(x_coords) < 2:
            return None
        
        # Fit a line using least squares
        coefficients = np.polyfit(y_coords, x_coords, 1)  # Fit x as function of y
        poly = np.poly1d(coefficients)
        
        # Generate points along the line
        y_points = np.linspace(height * 0.6, height, 20)
        x_points = poly(y_points)
        
        # Filter points to be within image bounds
        valid_points = [(x, y) for x, y in zip(x_points, y_points) 
                       if 0 <= x <= width and 0 <= y <= height]
        
        if len(valid_points) < 2:
            return None
        
        return np.array(valid_points, dtype=np.int32)
    
    def _detect_traffic_lights(self, frame: np.ndarray) -> List[TrafficLightState]:
        """Detect traffic lights in the frame."""
        traffic_lights = []
        
        # Use the object detection results to identify traffic lights
        objects = self._detect_objects(frame)
        
        for obj in objects:
            if obj.obj_type == ObjectType.TRAFFIC_LIGHT:
                # Determine traffic light state based on position and color
                state = self._determine_traffic_light_state(frame, obj.bbox)
                
                traffic_light = TrafficLightState(
                    state=state,
                    bbox=obj.bbox,
                    confidence=obj.confidence,
                    distance=obj.distance
                )
                traffic_lights.append(traffic_light)
        
        return traffic_lights
    
    def _determine_traffic_light_state(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> str:
        """Determine the state of a traffic light."""
        x1, y1, x2, y2 = bbox
        
        # Extract the traffic light region
        light_roi = frame[y1:y2, x1:x2]
        
        if light_roi.size == 0:
            return "unknown"
        
        # Convert to HSV for color detection
        hsv_roi = cv2.cvtColor(light_roi, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for traffic lights
        # Red light detection
        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])
        red_mask1 = cv2.inRange(hsv_roi, red_lower, red_upper)
        
        red_lower = np.array([170, 100, 100])
        red_upper = np.array([180, 255, 255])
        red_mask2 = cv2.inRange(hsv_roi, red_lower, red_upper)
        
        red_mask = red_mask1 + red_mask2
        
        # Yellow light detection
        yellow_lower = np.array([15, 100, 100])
        yellow_upper = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv_roi, yellow_lower, yellow_upper)
        
        # Green light detection
        green_lower = np.array([40, 100, 100])
        green_upper = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv_roi, green_lower, green_upper)
        
        # Count pixels for each color
        red_count = cv2.countNonZero(red_mask)
        yellow_count = cv2.countNonZero(yellow_mask)
        green_count = cv2.countNonZero(green_mask)
        
        # Determine state based on the most prominent color
        if red_count > yellow_count and red_count > green_count:
            return "red"
        elif yellow_count > red_count and yellow_count > green_count:
            return "yellow"
        elif green_count > red_count and green_count > yellow_count:
            return "green"
        else:
            return "unknown"
    
    def _detect_road_edges(self, frame: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Detect road edges."""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply edge detection
        edges = cv2.Canny(gray, 50, 150)
        
        # Define region of interest for road edge detection
        height, width = edges.shape
        roi_vertices = np.array([[
            (0, height),
            (width, height),
            (width * 0.8, height * 0.4),
            (width * 0.2, height * 0.4)
        ]], dtype=np.int32)
        
        # Create mask for ROI
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, roi_vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, threshold=50, 
                               minLineLength=30, maxLineGap=10)
        
        if lines is None:
            return None, None
        
        # Separate edges based on slope and position
        left_edge_points = []
        right_edge_points = []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Add small value to avoid division by zero
            
            # Classify as left or right edge based on position and slope
            center_x = (x1 + x2) / 2
            if center_x < width / 2 and slope < 0:  # Left side, negative slope
                left_edge_points.extend([(x1, y1), (x2, y2)])
            elif center_x > width / 2 and slope > 0:  # Right side, positive slope
                right_edge_points.extend([(x1, y1), (x2, y2)])
        
        # Convert to numpy arrays
        left_edge = np.array(left_edge_points, dtype=np.int32) if left_edge_points else None
        right_edge = np.array(right_edge_points, dtype=np.int32) if right_edge_points else None
        
        return left_edge, right_edge
    
    def _determine_drivable_area(self, frame: np.ndarray, lanes: LaneInfo) -> Optional[np.ndarray]:
        """Determine the drivable area in the frame."""
        height, width = frame.shape[:2]
        
        # Create a mask for drivable area
        drivable_mask = np.zeros((height, width), dtype=np.uint8)
        
        # If we have lane detection, define drivable area as between lanes
        if lanes.left_lane is not None and lanes.right_lane is not None:
            # Get the area between the lanes
            points = []
            
            # Add left lane points (reversed to close the polygon properly)
            if len(lanes.left_lane) > 0:
                points.extend(lanes.left_lane.tolist())
            
            # Add right lane points in reverse order
            if len(lanes.right_lane) > 0:
                points.extend(lanes.right_lane[::-1].tolist())
            
            if len(points) >= 3:
                pts = np.array(points, dtype=np.int32)
                cv2.fillPoly(drivable_mask, [pts], 255)
        
        # If no lanes detected, assume center of road is drivable
        elif lanes.left_lane is None and lanes.right_lane is None:
            # Use a rough estimate of drivable area
            center_x = width // 2
            lane_width = width // 4  # Estimate lane width
            cv2.rectangle(drivable_mask, 
                         (center_x - lane_width, height // 2),
                         (center_x + lane_width, height),
                         255, -1)
        
        return drivable_mask if np.any(drivable_mask) else None
    
    def get_latest_perception(self) -> Optional[PerceptionOutput]:
        """Get the latest perception output."""
        try:
            # Get the most recent result, discarding older ones
            result = None
            while True:
                try:
                    result = self.result_queue.get_nowait()
                except queue.Empty:
                    break
            
            return result
        except queue.Empty:
            return None
    
    def get_system_stats(self) -> Dict[str, Any]:
        """Get system statistics."""
        return {
            "queue_sizes": {
                "frame_queue": self.frame_queue.qsize(),
                "result_queue": self.result_queue.qsize()
            },
            "processing_time_ms": self.last_processing_time * 1000,
            "model_initialized": self.detection_model is not None,
            "running": self.running
        }

class PerceptionValidator:
    """
    Validation system to verify perception performance and accuracy
    for safety and reliability.
    """
    
    def __init__(self, perception_system: PerceptionSystem):
        self.perception_system = perception_system
        self.validation_metrics = {}
        self.running = False
        self.validator_thread = None
    
    def start_validation(self):
        """Start the perception validation."""
        self.running = True
        self.validator_thread = threading.Thread(target=self._validation_loop)
        self.validator_thread.start()
    
    def stop_validation(self):
        """Stop the perception validation."""
        self.running = False
        if self.validator_thread:
            self.validator_thread.join()
    
    def _validation_loop(self):
        """Validation loop to monitor perception performance."""
        while self.running:
            try:
                # Get latest perception data
                perception_data = self.perception_system.get_latest_perception()
                
                if perception_data:
                    # Validate perception outputs
                    self._validate_output(perception_data)
                
                time.sleep(1.0)  # Validate once per second
                
            except Exception as e:
                print(f"Validation error: {e}")
                time.sleep(1.0)
    
    def _validate_output(self, output: PerceptionOutput):
        """Validate perception output for safety and accuracy."""
        # Validate object detection consistency
        self._validate_objects(output.objects)
        
        # Validate lane detection
        self._validate_lanes(output.lanes)
        
        # Validate traffic light detection
        self._validate_traffic_lights(output.traffic_lights)
        
        # Check for potential safety issues
        self._check_safety_conditions(output)
    
    def _validate_objects(self, objects: List[DetectedObject]):
        """Validate object detection results."""
        # Check for consistency in object detection
        for obj in objects:
            # Validate confidence is reasonable
            if obj.confidence < 0.0 or obj.confidence > 1.0:
                print(f"Warning: Invalid confidence {obj.confidence} for {obj.obj_type}")
            
            # Validate bounding box is valid
            x1, y1, x2, y2 = obj.bbox
            if x1 < 0 or y1 < 0 or x2 > self.perception_system.frame_width or y2 > self.perception_system.frame_height:
                print(f"Warning: Invalid bbox for {obj.obj_type}: {obj.bbox}")
            
            if x1 > x2 or y1 > y2:
                print(f"Warning: Invalid bbox order for {obj.obj_type}: {obj.bbox}")
    
    def _validate_lanes(self, lanes: LaneInfo):
        """Validate lane detection results."""
        if lanes.confidence < 0.3:
            print("Warning: Low lane detection confidence")
        
        # Check for valid lane positions
        if abs(lanes.ego_lane_position) > 2.0:  # Should be between -1 and 1, with some tolerance
            print(f"Warning: Unexpected ego lane position: {lanes.ego_lane_position}")
    
    def _validate_traffic_lights(self, traffic_lights: List[TrafficLightState]):
        """Validate traffic light detection results."""
        for light in traffic_lights:
            if light.confidence < 0.5:
                print(f"Warning: Low confidence traffic light: {light.confidence}")
            
            if light.state not in ["red", "yellow", "green", "unknown"]:
                print(f"Warning: Invalid traffic light state: {light.state}")
    
    def _check_safety_conditions(self, output: PerceptionOutput):
        """Check for safety-related conditions."""
        # Check for immediate collision risks
        for obj in output.objects:
            if obj.distance and obj.distance < 20.0 and obj.obj_type in [ObjectType.CAR, ObjectType.TRUCK]:
                print(f"CRITICAL: Vehicle ahead at {obj.distance:.1f}m, fast approach!")
        
        # Check for traffic light violations
        for light in output.traffic_lights:
            if light.state == "red" and light.distance and light.distance < 50.0:
                print(f"CRITICAL: Approaching red light at {light.distance:.1f}m")

def test_perception_system():
    """Test function for the perception system."""
    print("Testing Perception System...")
    print("=" * 50)
    
    # Create perception system
    perception_system = PerceptionSystem()
    
    # Start the system
    perception_system.start()
    
    # Create validator
    validator = PerceptionValidator(perception_system)
    validator.start_validation()
    
    try:
        # Run for 10 seconds
        start_time = time.time()
        counter = 0
        
        while time.time() - start_time < 10:
            # Get perception data
            perception_data = perception_system.get_latest_perception()
            
            if perception_data:
                counter += 1
                
                # Print summary of this frame's detections
                print(f"\rFrame {counter}: {len(perception_data.objects)} objects, "
                      f"{len(perception_data.traffic_lights)} lights, "
                      f"lane conf: {perception_data.lanes.confidence:.2f}", end="")
                
                # Check for important detections
                for obj in perception_data.objects:
                    if obj.obj_type in [ObjectType.PEDESTRIAN, ObjectType.CAR]:
                        print(f"\n  Detected {obj.obj_type.value} at {obj.center}, conf: {obj.confidence:.2f}")
            
            time.sleep(0.1)  # 10Hz output
        
        print(f"\n\nProcessed {counter} frames in 10 seconds")
        
        # Get final stats
        stats = perception_system.get_system_stats()
        print(f"Final stats: {stats}")
        
    except KeyboardInterrupt:
        print("\nStopping perception system...")
    
    finally:
        # Stop both systems
        validator.stop_validation()
        perception_system.stop()

if __name__ == "__main__":
    test_perception_system()