"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
from typing import Optional
from openpilot.common.params import Params


class NavigationInterface:
  """Interface for setting and managing navigation destinations."""
  
  def __init__(self):
    self.params = Params()
    
  def set_destination(self, latitude: float, longitude: float, title: str = "Destination") -> bool:
    """
    Set a navigation destination.
    
    Args:
        latitude: Latitude in decimal degrees
        longitude: Longitude in decimal degrees
        title: Optional title for the destination
        
    Returns:
        True if destination was set successfully, False otherwise
    """
    try:
      destination_data = {
        "latitude": latitude,
        "longitude": longitude,
        "title": title,
        "timestamp": self._get_timestamp()
      }
      
      self.params.put("NavDestinationRequest", json.dumps(destination_data))
      return True
    except Exception:
      return False
  
  def clear_destination(self):
    """Clear the current destination."""
    self.params.delete("NavDestination")
    self.params.delete("NavDestinationRequest")
  
  def get_current_destination(self) -> Optional[dict]:
    """Get the current destination if set."""
    dest_str = self.params.get("NavDestination")
    if dest_str:
      try:
        return json.loads(dest_str)
      except json.JSONDecodeError:
        return None
    return None
  
  def is_navigation_active(self) -> bool:
    """Check if navigation is currently active."""
    nav_status_str = self.params.get("NavStatus")
    if nav_status_str:
      try:
        nav_status = json.loads(nav_status_str)
        return nav_status.get("active", False)
      except json.JSONDecodeError:
        pass
    return False
  
  def _get_timestamp(self) -> float:
    """Get current timestamp."""
    import time
    return time.time()


# Convenience function for setting destination
def set_navigation_destination(latitude: float, longitude: float, title: str = "Destination") -> bool:
  """Convenience function to set navigation destination."""
  nav_interface = NavigationInterface()
  return nav_interface.set_destination(latitude, longitude, title)


# Example usage:
if __name__ == "__main__":
  # Example: Set destination to Times Square, New York
  success = set_navigation_destination(40.7580, -73.9855, "Times Square, NY")
  if success:
    print("Destination set successfully!")
  else:
    print("Failed to set destination")
  
  # You can also use the class directly
  nav_interface = NavigationInterface()
  current_dest = nav_interface.get_current_destination()
  print(f"Current destination: {current_dest}")