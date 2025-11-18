#!/usr/bin/env python3
"""
Demonstration script for the enhanced validation system in Sunnypilot.
This script shows how the enhanced validation metrics work in practice.
"""
import sys
import os

# Add the project root to the path so imports work correctly
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from selfdrive.modeld.enhanced_vision import EnhancedVisionProcessor


def demonstrate_enhanced_validation():
    """Demonstrate the enhanced validation system."""
    print("Sunnypilot Enhanced Validation System Demo")
    print("=" * 50)
    
    processor = EnhancedVisionProcessor()
    
    print("\n1. Testing with empty model output (worst case scenario):")
    empty_output = {}
    validation_metrics = processor.validate_model_outputs(empty_output)
    
    print(f"   Lead confidence avg: {validation_metrics['lead_confidence_avg']:.3f}")
    print(f"   Lane confidence avg: {validation_metrics['lane_confidence_avg']:.3f}")
    print(f"   Lead count: {validation_metrics['lead_count']}")
    print(f"   Lane count: {validation_metrics['lane_count']}")
    print(f"   Overall confidence: {validation_metrics['overall_confidence']:.3f}")
    print(f"   Safety score: {validation_metrics['safety_score']:.3f}")
    print(f"   System should engage: {validation_metrics['system_should_engage']}")
    
    print("\n2. Testing with good model output (ideal scenario):")
    
    # Create mock objects with good confidence
    class MockLead:
        def __init__(self, prob):
            self.prob = prob

    class MockLane:
        def __init__(self, prob):
            self.prob = prob
        
        @property
        def points(self):
            return [10, 20, 30, 40, 50]  # Mock points for lane separation consistency

    good_output = {
        'leads_v3': [MockLead(0.9), MockLead(0.85)],
        'lane_lines': [MockLane(0.9), MockLane(0.85), MockLane(0.8), MockLane(0.95)],
        'road_edges': [MockLane(0.8), MockLane(0.75)]
    }
    
    validation_metrics_good = processor.validate_model_outputs(good_output)
    
    print(f"   Lead confidence avg: {validation_metrics_good['lead_confidence_avg']:.3f}")
    print(f"   Lane confidence avg: {validation_metrics_good['lane_confidence_avg']:.3f}")
    print(f"   Lane separation consistency: {validation_metrics_good['lane_separation_consistency']:.3f}")
    print(f"   Road edge confidence avg: {validation_metrics_good['road_edge_confidence_avg']:.3f}")
    print(f"   Lead count: {validation_metrics_good['lead_count']}")
    print(f"   Lane count: {validation_metrics_good['lane_count']}")
    print(f"   Overall confidence: {validation_metrics_good['overall_confidence']:.3f}")
    print(f"   Safety score: {validation_metrics_good['safety_score']:.3f}")
    print(f"   System should engage: {validation_metrics_good['system_should_engage']}")
    
    print("\n3. Testing safety thresholds:")
    
    # Create mock objects with poor confidence
    poor_output = {
        'leads_v3': [MockLead(0.1), MockLead(0.15)],
        'lane_lines': [MockLane(0.2), MockLane(0.15), MockLane(0.1), MockLane(0.25)]
    }
    
    validation_metrics_poor = processor.validate_model_outputs(poor_output)
    
    print(f"   Lead confidence avg: {validation_metrics_poor['lead_confidence_avg']:.3f} (low)")
    print(f"   Lane confidence avg: {validation_metrics_poor['lane_confidence_avg']:.3f} (low)")
    print(f"   Safety score: {validation_metrics_poor['safety_score']:.3f} (penalized due to low confidence)")
    print(f"   System should engage: {validation_metrics_poor['system_should_engage']} (should be False due to low confidence)")
    
    print("\n4. Summary of enhancements:")
    print("   - More comprehensive validation metrics")
    print("   - Safety scoring based on confidence thresholds")
    print("   - Lane separation consistency checking")
    print("   - System engagement decision logic")
    print("   - Road edge validation")
    print("   - Better overall confidence calculation with multiple factors")
    
    print("\nThis enhancement significantly improves the safety and reliability of the Sunnypilot system")
    print("by providing more detailed validation of neural network outputs and better decision-making")
    print("for when to engage or disengage the automated driving system.")


if __name__ == "__main__":
    demonstrate_enhanced_validation()