@0xbf82653406533946;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("sunnypilot::cereal");

# Validation metrics for enhanced vision processing
struct ValidationMetrics {
  # Lead detection confidence metrics
  leadConfidenceAvg @0 :Float32;
  leadConfidenceMax @1 :Float32;
  
  # Lane detection confidence metrics
  laneConfidenceAvg @2 :Float32;
  
  # Overall system confidence
  overallConfidence @3 :Float32;
  
  # Timestamp of when metrics were computed
  timestampMonoTime @4 :UInt64;
  
  # Additional validation metrics
  isValid @5 :Bool;
  confidenceThreshold @6 :Float32;
}