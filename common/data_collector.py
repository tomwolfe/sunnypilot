"""
Simple Data Collection for sunnypilot
Essential data collection with minimal complexity
"""
import json
import time
from pathlib import Path
from typing import Dict, Any

from openpilot.common.swaglog import cloudlog


class SimpleDataCollector:
  """Simple data collection system"""
  
  def __init__(self, output_dir: str = "/data/sunnypilot_metrics"):
    self.output_dir = Path(output_dir)
    self.output_dir.mkdir(parents=True, exist_ok=True)
    
    self.collection_enabled = True
    
  def collect_data(self, data_type: str, data: Dict[str, Any]):
    """Collect data point"""
    if not self.collection_enabled:
      return
    
    try:
      filename = self.output_dir / f"{data_type}_{int(time.time())}.json"
      with open(filename, 'w') as f:
        json.dump(data, f)
    except Exception as e:
      cloudlog.error(f"Error collecting data: {e}")


# Global instance
data_collector = SimpleDataCollector()