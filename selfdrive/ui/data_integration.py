"""
Real-time Data Integration for Sunnypilot UI
Connects to existing system services for live metrics and data visualization
"""
import pyray as rl
import time
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from enum import Enum

from cereal import messaging, log
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.controls.lib.events import Events, ET
from openpilot.selfdrive.locationd.calibrationd import MIN_SPEED_FILTER
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.alertmanager import set_offroad_alert


class DataServiceType(Enum):
    """Types of data services available"""
    PERCEPTION = "perception"
    NAVIGATION = "navigation"
    SAFETY = "safety"
    HARDWARE = "hardware"
    CONTROLS = "controls"


@dataclass
class DataService:
    """Container for data service information"""
    name: str
    service: str
    frequency: float  # Hz
    required: bool
    callback: Optional[callable] = None


class DataIntegrationManager:
    """Manages integration with all system data services"""
    
    def __init__(self):
        # SubMaster for receiving messages
        self.sm = messaging.SubMaster([
            "modelV2", "controlsState", "selfdriveState", "deviceState", 
            "carState", "driverStateV2", "radarState", "liveCalibration",
            "navInstruction", "navRoute", "peripheralState", "driverMonitoringState",
            "liveLocationKalman", "longitudinalPlan", "lateralPlan", "roadCameraState"
        ])
        
        # PubMaster for sending messages
        self.pm = messaging.Publisher([
            "userFlag", "userEvent", "uiDebug", "roadCameraState"
        ])
        
        # Data service configurations
        self.data_services = [
            DataService("Perception", "modelV2", 20.0, True),
            DataService("Controls", "controlsState", 100.0, True),
            DataService("Selfdrive", "selfdriveState", 20.0, True),
            DataService("Device", "deviceState", 2.0, True),
            DataService("Car", "carState", 100.0, True),
            DataService("Navigation", "navInstruction", 5.0, False),
            DataService("Navigation Route", "navRoute", 0.5, False),
            DataService("Driver Monitoring", "driverStateV2", 10.0, True),
            DataService("Radar", "radarState", 20.0, False),
            DataService("Calibration", "liveCalibration", 2.0, False),
        ]
        
        # Data caches to store processed information
        self.perception_data = {}
        self.navigation_data = {}
        self.safety_data = {}
        self.hardware_data = {}
        self.controls_data = {}
        
        # Timestamps for data freshness
        self.last_update_times = {service.name: 0.0 for service in self.data_services}
        
        # Status tracking
        self.service_status = {service.name: {"connected": False, "last_update": 0.0, "errors": 0} 
                              for service in self.data_services}
        
        # Rate keeper for updates
        self.ratekeeper = Ratekeeper(100)  # 100Hz base update rate
        
        print("Data Integration Manager initialized")
    
    def update(self) -> Dict[str, Any]:
        """Update all data services and return current data"""
        current_time = time.time()
        
        # Update messaging system
        try:
            self.sm.update(0)
        except Exception as e:
            print(f"Error updating messaging system: {e}")
            return self._get_empty_data()
        
        # Process updated data
        self._process_perception_data()
        self._process_control_data()
        self._process_device_data()
        self._process_navigation_data()
        
        # Update service status
        for service in self.data_services:
            if self.sm.updated[service.service]:
                self.service_status[service.name]["connected"] = True
                self.service_status[service.name]["last_update"] = current_time
                self.last_update_times[service.name] = current_time
        
        # Return combined data for UI
        combined_data = {
            'perception': self.perception_data,
            'controls': self.controls_data,
            'navigation': self.navigation_data,
            'safety': self._get_safety_status(),
            'hardware': self._get_hardware_status(),
            'timestamp': current_time
        }
        
        return combined_data
    
    def _process_perception_data(self):
        """Process perception system data"""
        if self.sm.updated["modelV2"]:
            model_data = self.sm["modelV2"]
            
            # Extract lane information
            lanes = {
                'left': None,
                'left_far': None,
                'right_far': None,
                'right': None
            }
            
            if len(model_data.laneLines) >= 4:
                lanes['left'] = {
                    'points': [(point.x, point.y) for point in model_data.laneLines[0].points if point.x != 0 or point.y != 0],
                    'confidence': model_data.laneLines[0].prob,
                    'std': model_data.laneLines[0].std
                }
                lanes['left_far'] = {
                    'points': [(point.x, point.y) for point in model_data.laneLines[1].points if point.x != 0 or point.y != 0],
                    'confidence': model_data.laneLines[1].prob,
                    'std': model_data.laneLines[1].std
                }
                lanes['right_far'] = {
                    'points': [(point.x, point.y) for point in model_data.laneLines[2].points if point.x != 0 or point.y != 0],
                    'confidence': model_data.laneLines[2].prob,
                    'std': model_data.laneLines[2].std
                }
                lanes['right'] = {
                    'points': [(point.x, point.y) for point in model_data.laneLines[3].points if point.x != 0 or point.y != 0],
                    'confidence': model_data.laneLines[3].prob,
                    'std': model_data.laneLines[3].std
                }
            
            # Extract lead vehicle information
            leads = []
            for lead in model_data.leadsV3:
                if lead.prob > 0.5:  # Only high confidence leads
                    leads.append({
                        'x': lead.x[0],
                        'y': lead.y[0],
                        'v_ego': lead.v[0] if lead.v else 0.0,
                        'a_ego': lead.a[0] if lead.a else 0.0,
                        'prob': lead.prob
                    })
            
            self.perception_data = {
                'lanes': lanes,
                'leads': leads,
                'desire_state': list(model_data.meta.desireState) if model_data.meta else [],
                'longitudinal_plan': {
                    'speeds': list(model_data.position.speeds) if model_data.position else [],
                    'x': list(model_data.position.x) if model_data.position else []
                }
            }
    
    def _process_control_data(self):
        """Process control system data"""
        if self.sm.updated["controlsState"]:
            controls_state = self.sm["controlsState"]
            
            self.controls_data = {
                'enabled': controls_state.enabled,
                'state': controls_state.state,
                'cumLagMs': controls_state.cumLagMs,
                'canMonoTimes': list(controls_state.canMonoTimes),
                'lateralDebug': {
                    'active': controls_state.lateralDebug.active,
                    'steerAngle': controls_state.lateralDebug.steerAngle,
                    'steerRate': controls_state.lateralDebug.steerRate,
                },
                'longitudinalPlan': {
                    'speeds': list(controls_state.longitudinalPlan.speeds) if controls_state.longitudinalPlan else [],
                    'accels': list(controls_state.longitudinalPlan.accels) if controls_state.longitudinalPlan else []
                }
            }
        
        # Get selfdrive state for engagement status
        if self.sm.updated["selfdriveState"]:
            selfdrive_state = self.sm["selfdriveState"]
            self.controls_data['selfdrive_enabled'] = selfdrive_state.enabled
            self.controls_data['selfdrive_state'] = selfdrive_state.state
            self.controls_data['experimental_mode'] = selfdrive_state.experimentalMode
    
    def _process_device_data(self):
        """Process device/hardware data"""
        if self.sm.updated["deviceState"]:
            device_state = self.sm["deviceState"]
            
            self.hardware_data = {
                'cpu_usage': device_state.cpuPct,
                'cpu_usage_per_core': list(device_state.cpuCores) if device_state.cpuCores else [],
                'gpu_usage': device_state.gpuUsagePercent,
                'memory_usage': device_state.memoryUsagePercent,
                'power_draw': device_state.powerDraw,
                'fan_speed': device_state.fanSpeedPercent,
                'thermal_status': device_state.thermalStatus,
                'charging': device_state.charging,
                'usb_power': device_state.usbPowerMode,
                'battery_percent': device_state.batteryPercent,
                'battery_status': device_state.batteryStatus
            }
    
    def _process_navigation_data(self):
        """Process navigation system data"""
        if self.sm.updated["navInstruction"]:
            nav_instr = self.sm["navInstruction"]
            
            self.navigation_data = {
                'maneuver': nav_instr.maneuver,
                'distance': nav_instr.distance,
                'time': nav_instr.time,
                'speed_limit': nav_instr.speedLimit,
                'road_name': nav_instr.roadName,
                'description': nav_instr.description,
                'distance_remaining': nav_instr.distanceRemaining,
                'time_remaining': nav_instr.timeRemaining,
                'route_progress': nav_instr.routeProgress
            }
    
    def _get_safety_status(self) -> Dict[str, Any]:
        """Get current safety status"""
        # In a real implementation, this would connect to the safety validation system
        # For now, we'll derive safety status from other available data
        
        safety_status = {
            'critical_alerts': [],
            'warnings': [],
            'system_health': 'GOOD',  # GOOD, WARNING, CRITICAL
            'safety_check_passed': True,
            'allow_engage': True,
            'lat_acc_lim': 1.0,  # Lateral acceleration limit
            'lon_acc_lim': 1.0,  # Longitudinal acceleration limit
        }
        
        # Check for critical alerts from controls state
        if self.sm.updated["controlsState"]:
            controls_state = self.sm["controlsState"]
            if controls_state.alertText1:  # There's an alert
                if "CRITICAL" in controls_state.alertText1.upper() or "DANGER" in controls_state.alertText1.upper():
                    safety_status['critical_alerts'].append(controls_state.alertText1)
                    safety_status['system_health'] = 'CRITICAL'
                    safety_status['allow_engage'] = False
                else:
                    safety_status['warnings'].append(controls_state.alertText1)
        
        # Check car state for safety issues
        if self.sm.updated["carState"]:
            car_state = self.sm["carState"]
            
            # Check for safety-critical conditions
            if car_state.brakePressed:
                safety_status['warnings'].append("Brake pressed")
            if car_state.gasPressed:
                safety_status['warnings'].append("Gas pressed")
            if not car_state.cruiseState.enabled:
                safety_status['warnings'].append("Cruise not enabled")
        
        # Check device state thermal status
        if self.sm.updated["deviceState"]:
            device_state = self.sm["deviceState"]
            if device_state.thermalStatus == 4:  # ThermalStatus.red
                safety_status['warnings'].append("High temperature")
                if device_state.thermalStatus == 5:  # ThermalStatus.dangerous
                    safety_status['critical_alerts'].append("Dangerous temperature")
                    safety_status['allow_engage'] = False
        
        return safety_status
    
    def _get_hardware_status(self) -> Dict[str, Any]:
        """Get current hardware status"""
        return self.hardware_data
    
    def _get_empty_data(self) -> Dict[str, Any]:
        """Return empty data structure when update fails"""
        return {
            'perception': {},
            'controls': {},
            'navigation': {},
            'safety': {'system_health': 'UNKNOWN', 'allow_engage': False},
            'hardware': {},
            'timestamp': time.time()
        }
    
    def get_data(self, service_type: DataServiceType) -> Optional[Dict[str, Any]]:
        """Get data for a specific service type"""
        current_data = self._get_empty_data()
        
        # Update data first
        current_data = self.update()
        
        return current_data.get(service_type.value, {})
    
    def send_user_event(self, event_type: str, details: Dict[str, Any] = None):
        """Send user event to system"""
        msg = messaging.new_message('userEvent')
        msg.userEvent.type = event_type
        if details:
            for key, value in details.items():
                setattr(msg.userEvent, key, value)
        self.pm.send('userEvent', msg)
    
    def get_service_connectivity(self) -> Dict[str, bool]:
        """Get connectivity status for all services"""
        return {name: status["connected"] for name, status in self.service_status.items()}
    
    def get_data_freshness(self) -> Dict[str, float]:
        """Get how fresh the data is for each service (seconds since last update)"""
        current_time = time.time()
        return {name: current_time - timestamp for name, timestamp in self.last_update_times.items()}


class RealtimeMetricsCollector:
    """Collects and processes real-time metrics for UI display"""
    
    def __init__(self, integration_manager: DataIntegrationManager):
        self.integration_manager = integration_manager
        
        # Metrics history for graphs
        self.metrics_history = {
            'cpu': [],
            'ram': [],
            'power': [],
            'temp': [],
            'fps': [],
        }
        self.max_history_points = 200  # For graphing
        
        # Performance metrics
        self.fps = 0.0
        self.last_frame_time = 0.0
        self.frame_count = 0
        self.last_fps_update = time.time()
    
    def update_metrics(self):
        """Update metrics from data integration"""
        # Get current data
        data = self.integration_manager.update()
        
        # Extract hardware metrics
        hw_data = data.get('hardware', {})
        if hw_data:
            self._add_to_history('cpu', hw_data.get('cpu_usage', 0))
            self._add_to_history('ram', hw_data.get('memory_usage', 0))
            self._add_to_history('power', hw_data.get('power_draw', 0))
            self._add_to_history('temp', hw_data.get('thermal_status', 0))
    
    def _add_to_history(self, metric: str, value: float):
        """Add a value to the metrics history"""
        if metric not in self.metrics_history:
            self.metrics_history[metric] = []
        
        self.metrics_history[metric].append(float(value))
        
        # Keep only the last N points
        if len(self.metrics_history[metric]) > self.max_history_points:
            self.metrics_history[metric] = self.metrics_history[metric][-self.max_history_points:]
    
    def update_fps(self, frame_time: float):
        """Update FPS calculation"""
        self.last_frame_time = frame_time
        self.frame_count += 1
        
        current_time = time.time()
        if current_time - self.last_fps_update > 1.0:  # Update every second
            self.fps = self.frame_count / (current_time - self.last_fps_update)
            self._add_to_history('fps', self.fps)
            self.frame_count = 0
            self.last_fps_update = current_time
    
    def get_current_metrics(self) -> Dict[str, float]:
        """Get current metrics"""
        current_data = self.integration_manager.update()
        hw_data = current_data.get('hardware', {})
        
        return {
            'cpu': hw_data.get('cpu_usage', 0),
            'ram': hw_data.get('memory_usage', 0),
            'power': hw_data.get('power_draw', 0),
            'temp': hw_data.get('thermal_status', 0),
            'fps': self.fps,
            'latency': current_data.get('timestamp', 0) - time.time()
        }
    
    def get_metric_history(self, metric: str) -> List[float]:
        """Get historical data for a specific metric"""
        return self.metrics_history.get(metric, [])


class SafetyDataProcessor:
    """Processes safety-critical information for UI display"""
    
    def __init__(self, integration_manager: DataIntegrationManager):
        self.integration_manager = integration_manager
        
        # Safety state variables
        self.last_critical_alert = ""
        self.safety_level = 0  # 0-5 safety level
        self.safety_validated = False
        
        # Advanced safety validation connection (simulated)
        self.advanced_safety_active = True
    
    def update_safety_data(self) -> Dict[str, Any]:
        """Update and return safety-related data"""
        data = self.integration_manager.update()
        
        # Process safety data
        safety_data = data.get('safety', {})
        
        # Update safety level based on system state
        self.safety_level = self._calculate_safety_level(safety_data, data.get('controls', {}))
        
        # Check for critical alerts
        if safety_data.get('critical_alerts'):
            self.last_critical_alert = safety_data['critical_alerts'][-1]
        
        # Validate with advanced safety system (simulated)
        self.safety_validated = self._validate_with_advanced_system()
        
        return {
            'safety_level': self.safety_level,
            'safety_validated': self.safety_validated,
            'allow_engage': safety_data.get('allow_engage', False),
            'critical_alerts': safety_data.get('critical_alerts', []),
            'warnings': safety_data.get('warnings', []),
            'system_health': safety_data.get('system_health', 'UNKNOWN'),
            'lat_acc_lim': safety_data.get('lat_acc_lim', 1.0),
            'lon_acc_lim': safety_data.get('lon_acc_lim', 3.0),
            'last_critical_alert': self.last_critical_alert
        }
    
    def _calculate_safety_level(self, safety_data: Dict, controls_data: Dict) -> int:
        """Calculate safety level based on various factors"""
        level = 5  # Start with highest safety level
        
        # Reduce level for warnings
        level -= len(safety_data.get('warnings', [])) * 0.5
        
        # Reduce level for critical alerts
        level -= len(safety_data.get('critical_alerts', [])) * 2
        
        # Check if system is disabled
        if not safety_data.get('allow_engage', True):
            level -= 2
        
        # Check control state
        if controls_data.get('state') == 4:  # Emergency state
            level = 1
        
        # Ensure level is between 0 and 5
        return max(0, min(5, int(level)))
    
    def _validate_with_advanced_system(self) -> bool:
        """Simulate connection to advanced safety validation"""
        # In a real implementation, this would connect to the advanced_safety_validation.py system
        # For now, simulate validation based on system health
        data = self.integration_manager.update()
        safety_data = data.get('safety', {})
        
        # Return True if system health is GOOD and no critical alerts
        return (safety_data.get('system_health') == 'GOOD' and 
                not safety_data.get('critical_alerts'))


class PerceptionDataProcessor:
    """Processes perception data for visualization"""
    
    def __init__(self, integration_manager: DataIntegrationManager):
        self.integration_manager = integration_manager
        
        # Processed perception data cache
        self.processed_objects = []
        self.processed_lanes = []
        self.traffic_signs = []
        
        # Object tracking
        self.object_trackers = {}
        self.next_object_id = 0
    
    def update_perception_data(self) -> Dict[str, Any]:
        """Update and return perception data"""
        data = self.integration_manager.update()
        perception_data = data.get('perception', {})
        
        # Process lane information
        lanes = perception_data.get('lanes', {})
        self.processed_lanes = self._process_lane_data(lanes)
        
        # Process lead vehicles
        leads = perception_data.get('leads', [])
        self.processed_objects = self._process_lead_data(leads)
        
        # Calculate derived perception metrics
        perception_metrics = {
            'object_count': len(self.processed_objects),
            'lane_confidence': self._calculate_lane_confidence(lanes),
            'closest_object_distance': self._get_closest_object_distance(),
            'free_path_available': self._is_path_clear(),
            'processed_lanes': self.processed_lanes,
            'processed_objects': self.processed_objects
        }
        
        return perception_metrics
    
    def _process_lane_data(self, lanes: Dict) -> List[Dict]:
        """Process raw lane data into display format"""
        processed = []
        
        for side, lane_data in lanes.items():
            if lane_data and lane_data.get('points'):
                processed.append({
                    'side': side,
                    'points': lane_data['points'],
                    'confidence': lane_data.get('confidence', 0),
                    'reliability': 1.0 - lane_data.get('std', 0)
                })
        
        return processed
    
    def _process_lead_data(self, leads: List[Dict]) -> List[Dict]:
        """Process lead vehicle data into display format"""
        processed = []
        
        for lead in leads:
            if lead.get('prob', 0) > 0.5:  # Only high confidence leads
                processed.append({
                    'x': lead['x'],
                    'y': lead['y'],
                    'v_ego': lead.get('v_ego', 0),
                    'a_ego': lead.get('a_ego', 0),
                    'prob': lead['prob'],
                    'type': 'car',  # Currently all detected objects are treated as cars
                    'width': 2.0,
                    'length': 4.5,
                    'height': 1.5
                })
        
        return processed
    
    def _calculate_lane_confidence(self, lanes: Dict) -> float:
        """Calculate overall lane detection confidence"""
        if not lanes:
            return 0.0
        
        confidences = [lane.get('confidence', 0) for lane in lanes.values() if lane]
        if not confidences:
            return 0.0
        
        return sum(confidences) / len(confidences)
    
    def _get_closest_object_distance(self) -> float:
        """Get distance to closest object ahead"""
        closest_dist = float('inf')
        
        for obj in self.processed_objects:
            if obj['x'] < 2.0 and obj['x'] > -2.0 and obj['y'] > 0:  # In front of vehicle
                closest_dist = min(closest_dist, obj['y'])
        
        return closest_dist if closest_dist != float('inf') else -1.0
    
    def _is_path_clear(self) -> bool:
        """Check if path ahead is clear of obstacles"""
        # Check for objects in the path (within 1.5m laterally and up to 50m ahead)
        for obj in self.processed_objects:
            if abs(obj['x']) < 1.5 and 0 < obj['y'] < 50:
                return False  # Path not clear
        return True


class NavigationDataProcessor:
    """Processes navigation data for display"""
    
    def __init__(self, integration_manager: DataIntegrationManager):
        self.integration_manager = integration_manager
        
        # Processed navigation data
        self.current_instruction = ""
        self.distance_to_maneuver = 0.0
        self.route_progress = 0.0
        self.estimated_time = 0.0
    
    def update_navigation_data(self) -> Dict[str, Any]:
        """Update and return navigation data"""
        data = self.integration_manager.update()
        nav_data = data.get('navigation', {})
        
        # Update navigation state
        self.current_instruction = nav_data.get('maneuver', 'None')
        self.distance_to_maneuver = nav_data.get('distance', 0.0)
        self.route_progress = nav_data.get('route_progress', 0.0)
        self.estimated_time = nav_data.get('time_remaining', 0.0)
        
        return {
            'current_instruction': self.current_instruction,
            'distance_to_maneuver': self.distance_to_maneuver,
            'route_progress': self.route_progress,
            'estimated_time': self.estimated_time,
            'destination': nav_data.get('road_name', 'Unknown'),
            'description': nav_data.get('description', ''),
            'speed_limit': nav_data.get('speed_limit', 0.0),
            'is_navigating': bool(nav_data)
        }


def connect_to_advanced_safety_system() -> bool:
    """Connect to the advanced safety validation system"""
    # In a real implementation, this would connect to the advanced_safety_validation.py system
    # For now, we'll just check if the module exists and is accessible
    try:
        import advanced_safety_validation
        print("Connected to advanced safety validation system")
        return True
    except ImportError:
        print("Advanced safety validation system not available")
        return False


def connect_to_hardware_monitor() -> bool:
    """Connect to hardware monitoring system"""
    # In a real implementation, this would connect to the hardware monitoring system
    # For now, we'll just return True as the data comes through the deviceState message
    print("Connected to hardware monitoring through deviceState")
    return True


def connect_to_navigation_system() -> bool:
    """Connect to navigation system"""
    # In a real implementation, this would connect to the navigation system
    # For now, we'll just return True as the data comes through navInstruction and navRoute messages
    print("Connected to navigation system through navInstruction/navRoute")
    return True


def main():
    """Main function to test data integration"""
    print("Testing Real-time Data Integration...")
    
    # Initialize data integration
    integration_manager = DataIntegrationManager()
    metrics_collector = RealtimeMetricsCollector(integration_manager)
    safety_processor = SafetyDataProcessor(integration_manager)
    perception_processor = PerceptionDataProcessor(integration_manager)
    navigation_processor = NavigationDataProcessor(integration_manager)
    
    # Test connections
    print("Testing system connections...")
    safety_connected = connect_to_advanced_safety_system()
    hardware_connected = connect_to_hardware_monitor()
    nav_connected = connect_to_navigation_system()
    
    print(f"Safety system: {'Connected' if safety_connected else 'Not available'}")
    print(f"Hardware monitor: {'Connected' if hardware_connected else 'Not available'}")
    print(f"Navigation system: {'Connected' if nav_connected else 'Not available'}")
    
    print("\nStarting real-time data integration test...")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            # Update all data processors
            integration_manager.update()
            metrics_collector.update_metrics()
            safety_data = safety_processor.update_safety_data()
            perception_data = perception_processor.update_perception_data()
            navigation_data = navigation_processor.update_navigation_data()
            
            # Get current metrics
            current_metrics = metrics_collector.get_current_metrics()
            
            # Display key information
            print(f"\rCPU: {current_metrics.get('cpu', 0):.1f}% | "
                  f"RAM: {current_metrics.get('ram', 0):.1f}% | "
                  f"Power: {current_metrics.get('power', 0):.2f}W | "
                  f"FPS: {current_metrics.get('fps', 0):.1f} | "
                  f"Objects: {perception_data.get('object_count', 0)} | "
                  f"Safety: L{int(safety_data.get('safety_level', 0))} | "
                  f"Nav: {navigation_data.get('current_instruction', 'None')}", end="", flush=True)
            
            time.sleep(0.1)  # Update 10 times per second
            
    except KeyboardInterrupt:
        print("\nData integration test stopped by user")


if __name__ == "__main__":
    main()