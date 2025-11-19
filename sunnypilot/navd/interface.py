"""
Simple Navigation Interface for Sunnypilot
Essential navigation functionality
"""
from typing import Dict, List, Tuple, Optional


class NavigationInterface:
  """Simple navigation interface class"""
  
  def __init__(self):
    self.destination: Optional[Tuple[float, float]] = None
    self.route: List[Dict] = []
    self.current_step: int = 0
  
  def set_destination(self, lat: float, lon: float):
    """Set navigation destination"""
    self.destination = (lat, lon)
    # In a real implementation, this would call routing APIs
  
  def get_next_instruction(self) -> Optional[Dict]:
    """Get next navigation instruction"""
    if self.route and self.current_step < len(self.route):
      return self.route[self.current_step]
    return None
  
  def update_position(self, lat: float, lon: float):
    """Update current position"""
    # In a real implementation, this would update current step based on position
    pass