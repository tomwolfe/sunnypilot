#!/usr/bin/env python3
"""
Multi-camera Object Tracking Visualization for Sunnypilot
Provides visualization tools for tracking results across multiple cameras
"""
import numpy as np
import cv2
from typing import Dict, List, Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import matplotlib.patches as patches
from collections import defaultdict
import time
import os


class MultiCameraTrackingVisualizer:
    """
    Visualization tools for multi-camera object tracking
    """

    def __init__(self):
        # Define camera configurations for visualization
        self.camera_configs = {
            'front_center': {'position': (0, 0), 'color': 'blue', 'fov': 120},
            'front_left': {'position': (-0.3, 0), 'color': 'red', 'fov': 120},
            'front_right': {'position': (0.3, 0), 'color': 'red', 'fov': 120},
            'front_left_side': {'position': (-0.5, 0), 'color': 'orange', 'fov': 90},
            'front_right_side': {'position': (0.5, 0), 'color': 'orange', 'fov': 90},
            'rear_left_side': {'position': (-0.5, -1), 'color': 'purple', 'fov': 90},
            'rear_right_side': {'position': (0.5, -1), 'color': 'purple', 'fov': 90},
            'rear_center': {'position': (0, -1), 'color': 'green', 'fov': 120},
        }
        
        # Class colors for different object types
        self.class_colors = {
            'car': (255, 0, 0),        # Red
            'person': (0, 255, 0),     # Green
            'bicycle': (0, 0, 255),    # Blue
            'truck': (255, 255, 0),    # Cyan
            'traffic light': (255, 0, 255),  # Magenta
            'sign': (0, 255, 255),     # Yellow
            'unknown': (128, 128, 128) # Gray
        }

    def visualize_3d_tracking(self, tracks: Dict[str, 'TrackedObject'], camera_data: Dict[str, np.ndarray], 
                            save_path: Optional[str] = None):
        """
        Visualize 3D tracking results in a top-down view
        """
        fig, ax = plt.subplots(1, 1, figsize=(12, 8))
        
        # Draw camera positions
        for cam_name, config in self.camera_configs.items():
            x, y = config['position']
            ax.plot(x, y, 's', color=config['color'], markersize=15, label=f'{cam_name}')
            ax.text(x, y - 0.05, cam_name, ha='center', va='top', fontsize=8)
        
        # Draw FOV for each camera
        for cam_name, config in self.camera_configs.items():
            x, y = config['position']
            fov = config['fov']
            
            # Draw a simplified FOV as a cone
            fov_rad = np.radians(fov / 2)
            # Draw left and right boundaries of FOV
            length = 0.5
            left_x = x + length * np.cos(fov_rad)
            left_y = y + length * np.sin(fov_rad)
            right_x = x + length * np.cos(-fov_rad)
            right_y = y + length * np.sin(-fov_rad)
            
            # Draw FOV polygon
            fov_x = [x, left_x, right_x]
            fov_y = [y, left_y, right_y]
            ax.fill(fov_x, fov_y, alpha=0.1, color=config['color'])
        
        # Draw tracked objects
        for track_id, track in tracks.items():
            x, y, z = track.state[:3]  # Position from state vector
            confidence = track.confidence
            
            # Color based on object class
            class_name = track.class_name
            color = self.class_colors.get(class_name, self.class_colors['unknown'])
            color = tuple(c/255.0 for c in color)  # Normalize for matplotlib
            
            # Draw object position
            ax.plot(x, y, 'o', color=color, markersize=10, alpha=confidence)
            
            # Draw object trajectory from history
            if len(track.detection_history) > 1:
                history_x = []
                history_y = []
                for det in track.detection_history:
                    # Get the 3D position from the detection
                    if 'position' in det:
                        history_x.append(det['position'][0])
                        history_y.append(det['position'][1])
                
                if len(history_x) > 1:
                    ax.plot(history_x, history_y, '--', color=color, alpha=0.5, linewidth=1)
            
            # Draw object ID and confidence
            ax.text(x, y + 0.02, f'{track.class_name}\n{confidence:.2f}', 
                   ha='center', va='bottom', fontsize=8, 
                   bbox=dict(boxstyle="round,pad=0.2", facecolor=color, alpha=0.3))
        
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.set_title('Multi-Camera 3D Tracking Visualization')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        # Set plot limits to show the relevant area
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.0)
        
        # Add legend
        legend_elements = [plt.Line2D([0], [0], marker='s', color='w', 
                                    markerfacecolor=config['color'], 
                                    markersize=10, label=cam_name) 
                          for cam_name, config in self.camera_configs.items()]
        ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.15, 1))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
        
        plt.show()

    def visualize_camera_view(self, camera_name: str, image: np.ndarray, 
                            detections: List[Dict], tracks: Dict[str, 'TrackedObject'],
                            save_path: Optional[str] = None):
        """
        Visualize a single camera view with detections and projected tracks
        """
        if image is None:
            print(f"No image available for {camera_name}")
            return
            
        # Create a copy of the image to draw on
        vis_img = image.copy()
        
        # Draw detections from this camera
        for det in detections:
            if det.get('camera') == camera_name:
                bbox = det.get('bbox', [0, 0, 0, 0])
                x, y, w, h = bbox
                conf = det.get('confidence', 0)
                class_name = det.get('class_name', 'unknown')
                
                # Draw bounding box
                pt1 = (int(x - w/2), int(y - h/2))
                pt2 = (int(x + w/2), int(y + h/2))
                color = self.class_colors.get(class_name, self.class_colors['unknown'])
                
                cv2.rectangle(vis_img, pt1, pt2, color, 2)
                
                # Draw confidence and class
                label = f'{class_name}: {conf:.2f}'
                cv2.putText(vis_img, label, (pt1[0], pt1[1] - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Find and draw projected tracks for this camera
        # This would involve projecting 3D tracked objects back to 2D camera space
        # For now, we'll just show the camera image with detections
        
        if save_path:
            cv2.imwrite(save_path, vis_img)
        
        # Display the image
        cv2.imshow(f'Camera View: {camera_name}', vis_img)
        cv2.waitKey(1)  # Allow time for display update

    def visualize_tracking_performance(self, tracking_stats: Dict[str, float]):
        """
        Visualize tracking performance metrics
        """
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))
        
        # Plot 1: Tracking accuracy over time
        ax1.bar(tracking_stats.keys(), tracking_stats.values(), color='skyblue')
        ax1.set_title('Tracking Performance Metrics')
        ax1.set_ylabel('Value')
        plt.setp(ax1.get_xticklabels(), rotation=45, ha="right")
        
        # Plot 2: Object type distribution
        object_types = ['car', 'person', 'bicycle', 'truck', 'traffic light']
        object_counts = [tracking_stats.get(f'{obj}_count', 0) for obj in object_types]
        ax2.pie(object_counts, labels=object_types, autopct='%1.1f%%', startangle=90)
        ax2.set_title('Detected Object Types Distribution')
        
        # Plot 3: Confidence distribution
        conf_ranges = ['<0.5', '0.5-0.7', '0.7-0.9', '>0.9']
        conf_counts = [
            tracking_stats.get('low_conf_count', 0),
            tracking_stats.get('med_conf_count', 0), 
            tracking_stats.get('high_conf_count', 0),
            tracking_stats.get('very_high_conf_count', 0)
        ]
        ax3.bar(conf_ranges, conf_counts, color=['red', 'orange', 'yellow', 'green'])
        ax3.set_title('Confidence Level Distribution')
        ax3.set_ylabel('Count')
        
        # Plot 4: Processing time
        time_metrics = ['avg_proc_time', 'max_proc_time', 'min_proc_time']
        time_values = [tracking_stats.get(tm, 0) for tm in time_metrics]
        ax4.bar(time_metrics, time_values, color='lightcoral')
        ax4.set_title('Processing Time Metrics (ms)')
        ax4.set_ylabel('Time (ms)')
        
        plt.tight_layout()
        plt.show()

    def generate_summary_report(self, tracks: Dict[str, 'TrackedObject'], 
                               processing_time: float) -> Dict[str, float]:
        """
        Generate a summary report of tracking performance
        """
        report = {}
        
        # Count different object types
        object_counts = defaultdict(int)
        total_confidence = 0
        confidence_levels = {'low': 0, 'med': 0, 'high': 0, 'vhigh': 0}
        
        for track_id, track in tracks.items():
            class_name = track.class_name
            object_counts[class_name] += 1
            
            total_confidence += track.confidence
            
            # Categorize confidence levels
            if track.confidence < 0.5:
                confidence_levels['low'] += 1
            elif track.confidence < 0.7:
                confidence_levels['med'] += 1
            elif track.confidence < 0.9:
                confidence_levels['high'] += 1
            else:
                confidence_levels['vhigh'] += 1
        
        # Calculate metrics
        report['total_tracks'] = len(tracks)
        report['avg_confidence'] = total_confidence / len(tracks) if tracks else 0
        report['processing_time_ms'] = processing_time * 1000  # Convert to ms
        
        # Add object type counts
        for obj_type, count in object_counts.items():
            report[f'{obj_type}_count'] = count
        
        # Add confidence level counts
        report['low_conf_count'] = confidence_levels['low']
        report['med_conf_count'] = confidence_levels['med']
        report['high_conf_count'] = confidence_levels['high']
        report['very_high_conf_count'] = confidence_levels['vhigh']
        
        # Performance metrics
        report['objects_per_second'] = len(tracks) / processing_time if processing_time > 0 else 0
        
        return report


def visualize_multi_camera_demo():
    """
    Demo function to demonstrate multi-camera tracking visualization
    """
    print("Multi-Camera Tracking Visualization Demo")
    print("=" * 50)
    
    # This would normally be connected to actual tracking system
    # For demo purposes, we'll create mock data
    visualizer = MultiCameraTrackingVisualizer()
    
    # Create mock tracking results
    class MockTrackedObject:
        def __init__(self, state, class_name, confidence):
            self.state = state
            self.class_name = class_name
            self.confidence = confidence
            self.detection_history = [
                {'position': state[:3]},
                {'position': state[:3] - np.array([0.1, 0.1, 0.0])},
                {'position': state[:3] - np.array([0.2, 0.2, 0.0])}
            ]
    
    # Create mock tracked objects
    tracks = {
        'track_1': MockTrackedObject(
            np.array([0.5, 5.0, 1.5, 0.1, 0.05, 0.0, 0.0, 0.0, 0.0]),  # [pos, vel, accel]
            'car', 0.9
        ),
        'track_2': MockTrackedObject(
            np.array([-1.0, 8.0, 1.7, -0.15, 0.03, 0.0, 0.0, 0.0, 0.0]),
            'person', 0.85
        ),
        'track_3': MockTrackedObject(
            np.array([1.5, 6.0, 1.2, 0.05, 0.08, 0.0, 0.0, 0.0, 0.0]),
            'bicycle', 0.8
        )
    }
    
    # Create mock camera data
    camera_data = {
        'front_center': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        'front_left': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        'front_right': np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
    }
    
    # Create mock detections
    detections = [
        {'bbox': [320, 240, 100, 150], 'camera': 'front_center', 'confidence': 0.9, 'class_name': 'car'},
        {'bbox': [200, 300, 50, 120], 'camera': 'front_center', 'confidence': 0.85, 'class_name': 'person'},
        {'bbox': [450, 350, 40, 80], 'camera': 'front_center', 'confidence': 0.8, 'class_name': 'bicycle'},
    ]
    
    # Generate summary report
    report = visualizer.generate_summary_report(tracks, 0.05)  # 50ms processing time
    print("Tracking Summary Report:")
    for key, value in report.items():
        print(f"  {key}: {value}")
    
    # Visualize 3D tracking
    print("\nGenerating 3D tracking visualization...")
    visualizer.visualize_3d_tracking(tracks, camera_data)
    
    # Visualize camera views
    print("\nGenerating camera view visualizations...")
    for cam_name, img in camera_data.items():
        visualizer.visualize_camera_view(cam_name, img, detections, tracks)
    
    # Visualize performance
    print("\nGenerating performance visualization...")
    visualizer.visualize_tracking_performance(report)
    
    # Wait for user to close windows
    print("\nPress any key in the image windows to continue...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    print("\nVisualization demo completed!")


if __name__ == "__main__":
    visualize_multi_camera_demo()