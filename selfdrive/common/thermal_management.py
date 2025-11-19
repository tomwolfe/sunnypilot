"""
Enhanced thermal and resource management for Sunnypilot
Predictive thermal management and dynamic resource allocation
"""

import time
import psutil
from typing import Dict, Any, Optional, Tuple
from cereal import log
import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE


class PredictiveThermalManager:
    """
    Enhanced thermal and resource management with predictive capabilities
    """
    
    def __init__(self):
        self.params = Params()
        self.last_update_time = time.time()
        
        # Thermal thresholds for different components
        self.thermal_thresholds = {
            'cpu': {'min': 30, 'max': 85, 'critical': 95},
            'gpu': {'min': 30, 'max': 90, 'critical': 100},
            'memory': {'min': 0, 'max': 80, 'critical': 90},  # percentage
            'power': {'min': 0, 'max': 85, 'critical': 95}    # percentage
        }
        
        # Performance scaling factors
        self.performance_scaling = 1.0
        self.last_performance_scale = 1.0
        
        # Predictive models
        self.thermal_history = []
        self.max_history_len = 50
        
        # Resource allocation tracking
        self.resource_allocations = {}
        self.process_load_history = {}
        
        # Initialize messaging
        try:
            self.sm = messaging.SubMaster(['deviceState', 'carState', 'modelV2'], 
                                         ignore_alive=True)
        except Exception as e:
            cloudlog.warning(f"Could not initialize SubMaster in PredictiveThermalManager: {e}")
            self.sm = None
    
    def update_thermal_status(self, device_state: log.DeviceState) -> Dict[str, Any]:
        """
        Update thermal status and return enhanced thermal metrics
        """
        current_time = time.time()
        time_delta = current_time - self.last_update_time
        self.last_update_time = current_time

        try:
            # Get current thermal data
            thermal_data = {
                'cpu_temp': getattr(device_state, 'cpuTempC', 0),
                'gpu_temp': getattr(device_state, 'gpuTempC', 0),
                'memory_percent': psutil.virtual_memory().percent,
                'cpu_percent': psutil.cpu_percent(),
                'thermal_status': getattr(device_state, 'thermalStatus', 0)
            }

            # Add to history for predictive analysis
            self.thermal_history.append({
                'timestamp': current_time,
                'data': thermal_data.copy()
            })
            if len(self.thermal_history) > self.max_history_len:
                self.thermal_history.pop(0)

            # Predict future thermal load
            predicted_thermal_load = self._predict_thermal_load()

            # Calculate thermal safety metrics
            thermal_metrics = self._calculate_thermal_metrics(thermal_data, predicted_thermal_load)

            # Adjust performance scaling based on thermal state
            self._adjust_performance_scaling(thermal_metrics)

            return thermal_metrics
        except Exception as e:
            cloudlog.error(f"Error updating thermal status: {e}")
            # Return safe defaults in case of error
            return {
                'current_cpu_temp': 30,
                'current_gpu_temp': 30,
                'current_memory_percent': 50,
                'current_cpu_percent': 30,
                'predicted_cpu_temp': 35,
                'thermal_trend': 'stable',
                'thermal_score': 1.0,
                'performance_scale': 1.0,
                'system_thermal_state': 'normal'
            }
    
    def _predict_thermal_load(self) -> Dict[str, float]:
        """Predict future thermal load based on historical data"""
        if len(self.thermal_history) < 3:
            return {'cpu_temp': 0.0, 'gpu_temp': 0.0, 'trend': 'stable'}
        
        # Simple linear prediction based on recent trends
        recent_data = self.thermal_history[-5:]  # Last 5 readings
        
        # Calculate trend for CPU temperature
        if len(recent_data) >= 2:
            first_temp = recent_data[0]['data']['cpu_temp']
            last_temp = recent_data[-1]['data']['cpu_temp']
            time_diff = recent_data[-1]['timestamp'] - recent_data[0]['timestamp']
            
            if time_diff > 0:
                temp_rate = (last_temp - first_temp) / time_diff
                # Predict next 30 seconds
                predicted_temp = last_temp + (temp_rate * 30)
            else:
                predicted_temp = last_temp
        else:
            predicted_temp = self.thermal_history[-1]['data']['cpu_temp']
        
        # Determine trend
        if predicted_temp > self.thermal_thresholds['cpu']['max']:
            trend = 'heating'
        elif predicted_temp < self.thermal_thresholds['cpu']['min']:
            trend = 'cooling'
        else:
            trend = 'stable'
        
        return {
            'cpu_temp': max(0, predicted_temp),
            'gpu_temp': 0.0,  # Simplified prediction
            'trend': trend
        }
    
    def _calculate_thermal_metrics(self, current_data: Dict, predicted_data: Dict) -> Dict[str, Any]:
        """Calculate comprehensive thermal metrics"""
        metrics = {
            'current_cpu_temp': current_data['cpu_temp'],
            'current_gpu_temp': current_data['gpu_temp'],
            'current_memory_percent': current_data['memory_percent'],
            'current_cpu_percent': current_data['cpu_percent'],
            'predicted_cpu_temp': predicted_data['cpu_temp'],
            'thermal_trend': predicted_data['trend'],
            'thermal_score': 1.0,  # 1.0 = good, 0.0 = critical
            'performance_scale': self.performance_scaling,
            'system_thermal_state': 'normal'
        }
        
        # Calculate individual component scores
        cpu_score = self._calculate_component_score(
            current_data['cpu_temp'], 
            self.thermal_thresholds['cpu']
        )
        
        memory_score = self._calculate_component_score(
            current_data['memory_percent'],
            self.thermal_thresholds['memory']
        )
        
        cpu_usage_score = self._calculate_component_score(
            current_data['cpu_percent'],
            {'min': 0, 'max': 70, 'critical': 90}  # Custom thresholds for CPU usage
        )
        
        # Overall thermal score is the minimum of individual scores
        metrics['thermal_score'] = min(cpu_score, memory_score, cpu_usage_score)
        
        # Determine system thermal state
        if metrics['thermal_score'] < 0.3:
            metrics['system_thermal_state'] = 'critical'
        elif metrics['thermal_score'] < 0.6:
            metrics['system_thermal_state'] = 'warning'
        elif metrics['thermal_score'] < 0.8:
            metrics['system_thermal_state'] = 'caution'
        else:
            metrics['system_thermal_state'] = 'normal'
        
        return metrics
    
    def _calculate_component_score(self, value: float, thresholds: Dict[str, float]) -> float:
        """
        Calculate thermal score for a single component (0.0 to 1.0)

        Args:
            value: Current value of the component to score
            thresholds: Dictionary containing 'min', 'max', and 'critical' thresholds

        Returns:
            float: Score between 0.0 (critical) and 1.0 (optimal)
        """
        # Validate thresholds to avoid division by zero
        min_val = thresholds.get('min', 0.0)
        max_val = thresholds.get('max', 100.0)
        critical_val = thresholds.get('critical', 110.0)

        if max_val <= min_val:
            cloudlog.error(f"Invalid thresholds: max ({max_val}) <= min ({min_val})")
            return 1.0  # Return safe default

        if critical_val <= max_val:
            cloudlog.error(f"Invalid thresholds: critical ({critical_val}) <= max ({max_val})")
            return 1.0  # Return safe default

        if value <= min_val:
            return 1.0
        elif value >= critical_val:
            return 0.0
        elif value <= max_val:
            # Between min and max: good range
            return 1.0 - ((value - min_val) / (max_val - min_val))
        else:
            # Between max and critical: warning range
            warning_score = (value - max_val) / (critical_val - max_val)
            return max(0.0, 0.3 * (1.0 - warning_score))  # Gracefully reduce to 0
    
    def _adjust_performance_scaling(self, thermal_metrics: Dict[str, Any]) -> None:
        """Adjust performance scaling based on thermal state"""
        current_scale = thermal_metrics['thermal_score']
        
        # Apply smoothing to prevent rapid scaling changes
        target_scale = max(0.3, min(1.0, current_scale))  # Clamp between 0.3 and 1.0
        self.performance_scaling = 0.9 * self.last_performance_scale + 0.1 * target_scale
        self.last_performance_scale = self.performance_scaling
    
    def get_resource_recommendations(self) -> Dict[str, Any]:
        """
        Get resource allocation recommendations based on current thermal state
        """
        recommendations = {
            'model_complexity': 'normal',  # 'low', 'normal', 'high'
            'camera_resolution': 'normal',
            'update_frequency': 20,  # Hz
            'memory_limit_mb': 800,  # 800MB limit under thermal stress
            'cpu_priority': 'normal'
        }
        
        thermal_state = self.thermal_history[-1]['data'] if self.thermal_history else None
        if thermal_state:
            if thermal_state['data']['cpu_temp'] > self.thermal_thresholds['cpu']['max']:
                recommendations['model_complexity'] = 'low'
                recommendations['camera_resolution'] = 'low'
                recommendations['update_frequency'] = 10
                recommendations['memory_limit_mb'] = 600
                recommendations['cpu_priority'] = 'low'
            elif thermal_state['data']['cpu_temp'] > self.thermal_thresholds['cpu']['min'] + 10:
                # Moderate temperature
                recommendations['model_complexity'] = 'normal'
                recommendations['camera_resolution'] = 'normal'
                recommendations['update_frequency'] = 15
                recommendations['memory_limit_mb'] = 700
                recommendations['cpu_priority'] = 'normal'
        
        return recommendations
    
    def get_current_performance_scale(self) -> float:
        """Get the current performance scaling factor (0.0 to 1.0)"""
        return self.performance_scaling
    
    def get_thermal_advice(self, current_task: str = "model_inference") -> Dict[str, Any]:
        """
        Get specific thermal advice for a current task
        """
        thermal_state = self.thermal_history[-1]['data'] if self.thermal_history else None
        if not thermal_state:
            return {'action': 'continue', 'delay': 0}
        
        advice = {'action': 'continue', 'delay': 0}
        
        # Check if we should reduce load based on thermal state
        if thermal_state['data']['cpu_temp'] > self.thermal_thresholds['cpu']['max'] * 0.9:
            advice['action'] = 'reduce_load'
            advice['delay'] = 0.05  # 50ms additional delay
        elif thermal_state['data']['memory_percent'] > self.thermal_thresholds['memory']['max'] * 0.9:
            advice['action'] = 'reduce_memory'
        
        return advice


class ResourceManager:
    """
    Resource manager for dynamic allocation and monitoring
    """
    
    def __init__(self):
        self.process_resources = {}
        self.max_memory_per_process = 100 * 1024 * 1024  # 100 MB in bytes
        self.thermal_manager = PredictiveThermalManager()
    
    def request_resources(self, process_id: str, priority: str = "normal", 
                         cpu_required: float = 1.0, memory_required: float = 50.0,
                         gpu_required: float = 0.0, duration_estimate: float = 0.1) -> Dict[str, Any]:
        """
        Request resources for a process with thermal awareness
        """
        thermal_advice = self.thermal_manager.get_thermal_advice(process_id)
        
        # Check if request can be fulfilled given thermal constraints
        if thermal_advice['action'] == 'reduce_load':
            # Reduce requested resources under thermal stress
            cpu_required = max(0.1, cpu_required * 0.7)
            memory_required = max(10.0, memory_required * 0.8)
            duration_estimate = min(0.2, duration_estimate * 1.5)
        
        allocation = {
            'process_id': process_id,
            'cpu_percent': min(cpu_required, 100.0),
            'memory_mb': min(memory_required, 1000.0),  # Cap at 1000MB
            'gpu_percent': min(gpu_required, 100.0),
            'priority': priority,
            'allocated_at': time.time(),
            'duration_estimate': duration_estimate,
            'thermal_compliance': thermal_advice
        }
        
        self.process_resources[process_id] = allocation
        return allocation
    
    def release_resources(self, process_id: str) -> bool:
        """Release resources for a process"""
        if process_id in self.process_resources:
            del self.process_resources[process_id]
            return True
        return False


# Global instances for use across the system
thermal_manager = PredictiveThermalManager()
resource_manager = ResourceManager()