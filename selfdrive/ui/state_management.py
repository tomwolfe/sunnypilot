#!/usr/bin/env python3
"""
Robust State Management and Lifecycle Handling for Sunnypilot UI System
Manages UI states, component lifecycle, and transitions between different driving modes
"""
import time
from enum import Enum
from typing import Dict, List, Callable, Any, Optional
from dataclasses import dataclass
from abc import ABC, abstractmethod


class UIState(Enum):
    """Represents different states of the UI"""
    BOOTING = "booting"
    IDLE = "idle"
    AWAITING_ENGAGEMENT = "awaiting_engagement"
    ENGAGED = "engaged"
    DISENGAGING = "disengaging"
    EMERGENCY = "emergency"
    SHUTTING_DOWN = "shutting_down"
    ERROR = "error"


class LifecycleState(Enum):
    """Lifecycle states of UI components"""
    UNINITIALIZED = "uninitialized"
    INITIALIZING = "initializing"
    INITIALIZED = "initialized"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    STOPPED = "stopped"
    ERROR = "error"


@dataclass
class StateTransition:
    """Represents a state transition with timing and validation"""
    from_state: UIState
    to_state: UIState
    timestamp: float
    valid: bool = True
    reason: str = ""


class StateValidator(ABC):
    """Abstract base class for state validators"""
    
    @abstractmethod
    def validate_transition(self, from_state: UIState, to_state: UIState, context: Dict[str, Any]) -> bool:
        """Validate if a transition from one state to another is allowed"""
        pass


class DrivingStateValidator(StateValidator):
    """Validates transitions based on driving safety requirements"""
    
    def validate_transition(self, from_state: UIState, to_state: UIState, context: Dict[str, Any]) -> bool:
        """Validate state transitions based on safety requirements"""
        # Define valid transitions
        valid_transitions = {
            UIState.BOOTING: [UIState.IDLE, UIState.ERROR],
            UIState.IDLE: [UIState.AWAITING_ENGAGEMENT, UIState.ERROR],
            UIState.AWAITING_ENGAGEMENT: [UIState.ENGAGED, UIState.IDLE, UIState.EMERGENCY, UIState.ERROR],
            UIState.ENGAGED: [UIState.DISENGAGING, UIState.EMERGENCY, UIState.ERROR],
            UIState.DISENGAGING: [UIState.IDLE, UIState.EMERGENCY, UIState.ERROR],
            UIState.EMERGENCY: [UIState.IDLE, UIState.ERROR],
            UIState.ERROR: [UIState.IDLE, UIState.SHUTTING_DOWN],
            UIState.SHUTTING_DOWN: []
        }
        
        # Check if the transition is in the allowed list
        if to_state in valid_transitions.get(from_state, []):
            # Additional safety checks
            if to_state == UIState.ENGAGED:
                # Check if engagement is safe
                safety_ok = context.get('safety_ok', True)
                sensors_ok = context.get('sensors_ok', True)
                return safety_ok and sensors_ok
            
            elif from_state == UIState.ENGAGED and to_state == UIState.DISENGAGING:
                # Ensure we're allowed to disengage safely
                return True  # Always allow disengagement for safety
            
            elif to_state == UIState.EMERGENCY:
                # Emergency state is always valid when needed
                return True
            
            return True
            
        return False


class UIStateManager:
    """Manages the overall UI state and coordinates state transitions"""
    
    def __init__(self):
        self.current_state = UIState.BOOTING
        self.previous_state = None
        self.state_validators: List[StateValidator] = [DrivingStateValidator()]
        self.transition_history: List[StateTransition] = []
        self.state_enter_times: Dict[UIState, float] = {}
        self.context = {}  # Context for validation
        self.listeners: List[Callable] = []  # State change listeners
        
        # Initialize the booting state
        self.state_enter_times[UIState.BOOTING] = time.time()
    
    def add_validator(self, validator: StateValidator):
        """Add a state validator"""
        self.state_validators.append(validator)
    
    def add_state_listener(self, callback: Callable[[UIState, UIState], None]):
        """Add a callback to be called on state changes"""
        self.listeners.append(callback)
    
    def set_context(self, key: str, value: Any):
        """Set validation context"""
        self.context[key] = value
    
    def get_context(self, key: str, default: Any = None):
        """Get validation context"""
        return self.context.get(key, default)
    
    def can_transition_to(self, new_state: UIState) -> bool:
        """Check if transition to new state is valid"""
        for validator in self.state_validators:
            if not validator.validate_transition(self.current_state, new_state, self.context):
                return False
        return True
    
    def transition_to(self, new_state: UIState, reason: str = "") -> bool:
        """Attempt to transition to a new state"""
        if not self.can_transition_to(new_state):
            transition = StateTransition(self.current_state, new_state, time.time(), False, 
                                       f"Transition not allowed: {self.current_state} -> {new_state}")
            self.transition_history.append(transition)
            return False
        
        # Execute the transition
        self.previous_state = self.current_state
        old_state = self.current_state
        self.current_state = new_state
        
        # Record the transition
        transition = StateTransition(old_state, new_state, time.time(), True, reason)
        self.transition_history.append(transition)
        
        # Update enter time for the new state
        self.state_enter_times[new_state] = time.time()
        
        # Notify listeners
        for listener in self.listeners:
            try:
                listener(old_state, new_state)
            except Exception as e:
                print(f"Error in state transition listener: {e}")
        
        return True
    
    def get_state_duration(self, state: UIState) -> float:
        """Get how long we've been in a particular state"""
        enter_time = self.state_enter_times.get(state)
        if enter_time is None:
            return 0.0
        
        if state == self.current_state:
            # Currently in this state, return time since enter
            return time.time() - enter_time
        else:
            # Find the transition that left this state and calculate duration
            for i, trans in enumerate(self.transition_history):
                if trans.from_state == state and trans.valid:
                    # Find when we left this state
                    for j in range(i+1, len(self.transition_history)):
                        if self.transition_history[j].from_state == state:
                            return self.transition_history[j].timestamp - enter_time
            return 0.0  # State was entered but not exited yet
    
    def is_in_state(self, state: UIState, min_duration: float = 0.0) -> bool:
        """Check if currently in a state for at least min_duration seconds"""
        if self.current_state != state:
            return False
        
        duration = self.get_state_duration(state)
        return duration >= min_duration
    
    def get_recent_transitions(self, count: int = 5) -> List[StateTransition]:
        """Get recent state transitions"""
        return self.transition_history[-count:]


class ComponentLifecycleManager:
    """Manages the lifecycle of UI components"""
    
    def __init__(self, state_manager: UIStateManager):
        self.state_manager = state_manager
        self.components: Dict[str, 'LifecycleComponent'] = {}
        self.component_states: Dict[str, LifecycleState] = {}
        
        # Register for state change notifications
        self.state_manager.add_state_listener(self._on_global_state_change)
    
    def register_component(self, name: str, component: 'LifecycleComponent'):
        """Register a component with the lifecycle manager"""
        self.components[name] = component
        self.component_states[name] = LifecycleState.UNINITIALIZED
    
    def initialize_component(self, name: str) -> bool:
        """Initialize a component"""
        if name not in self.components:
            return False
            
        try:
            self.component_states[name] = LifecycleState.INITIALIZING
            self.components[name].initialize()
            self.component_states[name] = LifecycleState.INITIALIZED
            return True
        except Exception as e:
            print(f"Error initializing component {name}: {e}")
            self.component_states[name] = LifecycleState.ERROR
            return False
    
    def start_component(self, name: str) -> bool:
        """Start a component"""
        if name not in self.components or self.component_states[name] != LifecycleState.INITIALIZED:
            return False
            
        try:
            self.component_states[name] = LifecycleState.STARTING
            self.components[name].start()
            self.component_states[name] = LifecycleState.RUNNING
            return True
        except Exception as e:
            print(f"Error starting component {name}: {e}")
            self.component_states[name] = LifecycleState.ERROR
            return False
    
    def stop_component(self, name: str) -> bool:
        """Stop a component"""
        if name not in self.components or self.component_states[name] != LifecycleState.RUNNING:
            return False
            
        try:
            self.component_states[name] = LifecycleState.STOPPING
            self.components[name].stop()
            self.component_states[name] = LifecycleState.STOPPED
            return True
        except Exception as e:
            print(f"Error stopping component {name}: {e}")
            self.component_states[name] = LifecycleState.ERROR
            return False
    
    def destroy_component(self, name: str) -> bool:
        """Destroy a component"""
        if name not in self.components:
            return False
            
        try:
            if self.component_states[name] in [LifecycleState.RUNNING, LifecycleState.STOPPED]:
                self.stop_component(name)
            
            self.components[name].destroy()
            del self.components[name]
            del self.component_states[name]
            return True
        except Exception as e:
            print(f"Error destroying component {name}: {e}")
            return False
    
    def update_component(self, name: str):
        """Update a component if it's running"""
        if (name in self.components and 
            self.component_states[name] == LifecycleState.RUNNING):
            try:
                self.components[name].update()
            except Exception as e:
                print(f"Error updating component {name}: {e}")
                self.component_states[name] = LifecycleState.ERROR
    
    def update_all_components(self):
        """Update all running components"""
        for name in list(self.components.keys()):  # Use list to avoid modification during iteration
            self.update_component(name)
    
    def _on_global_state_change(self, old_state: UIState, new_state: UIState):
        """Handle global state changes that may affect components"""
        # For certain state transitions, we may need to start/stop components
        if new_state == UIState.EMERGENCY:
            # In emergency state, stop non-critical components
            for name, state in self.component_states.items():
                if state == LifecycleState.RUNNING:
                    component = self.components[name]
                    if not getattr(component, 'critical_in_emergency', False):
                        self.stop_component(name)
        elif new_state == UIState.ENGAGED and old_state in [UIState.IDLE, UIState.AWAITING_ENGAGEMENT]:
            # When engaging, start driving-related components
            for name, state in self.component_states.items():
                if state == LifecycleState.STOPPED:
                    component = self.components[name]
                    if getattr(component, 'active_when_engaged', False):
                        self.start_component(name)
        elif new_state in [UIState.IDLE, UIState.DISENGAGING] and old_state == UIState.ENGAGED:
            # When disengaging, stop driving-related components
            for name, state in self.component_states.items():
                if state == LifecycleState.RUNNING:
                    component = self.components[name]
                    if getattr(component, 'active_when_engaged', False) and not getattr(component, 'critical', False):
                        self.stop_component(name)


class LifecycleComponent(ABC):
    """Abstract base class for lifecycle-managed components"""
    
    def __init__(self, name: str, critical: bool = False, active_when_engaged: bool = True):
        self.name = name
        self.critical = critical
        self.active_when_engaged = active_when_engaged
        self.critical_in_emergency = critical  # Critical components stay active in emergency by default
        self.initialized = False
    
    @abstractmethod
    def initialize(self):
        """Initialize the component"""
        pass
    
    @abstractmethod
    def start(self):
        """Start the component"""
        pass
    
    @abstractmethod
    def update(self):
        """Update the component"""
        pass
    
    @abstractmethod
    def stop(self):
        """Stop the component"""
        pass
    
    @abstractmethod
    def destroy(self):
        """Destroy the component"""
        pass


class StateAwareComponent(LifecycleComponent):
    """Base class for components that are aware of UI states"""
    
    def __init__(self, name: str, critical: bool = False, active_when_engaged: bool = True):
        super().__init__(name, critical, active_when_engaged)
        self.state_manager: Optional[UIStateManager] = None
        self.lifecycle_manager: Optional[ComponentLifecycleManager] = None
    
    def set_managers(self, state_manager: UIStateManager, lifecycle_manager: ComponentLifecycleManager):
        """Set the managers for this component"""
        self.state_manager = state_manager
        self.lifecycle_manager = lifecycle_manager
    
    def get_current_ui_state(self) -> Optional[UIState]:
        """Get the current UI state"""
        if self.state_manager:
            return self.state_manager.current_state
        return None
    
    def can_transition_to(self, new_state: UIState) -> bool:
        """Check if the global state can transition to a new state"""
        if self.state_manager:
            return self.state_manager.can_transition_to(new_state)
        return False
    
    def request_state_transition(self, new_state: UIState) -> bool:
        """Request a global state transition"""
        if self.state_manager:
            return self.state_manager.transition_to(new_state)
        return False


def create_default_state_manager() -> UIStateManager:
    """Create a default state manager with common configuration"""
    state_manager = UIStateManager()
    
    # Set some common context values
    state_manager.set_context('safety_ok', True)
    state_manager.set_context('sensors_ok', True)
    state_manager.set_context('system_ready', True)
    
    return state_manager


def main():
    """Main function to demonstrate state management"""
    print("Sunnypilot UI State Management System")
    
    # Create state manager
    state_manager = create_default_state_manager()
    
    # Show initial state
    print(f"Initial state: {state_manager.current_state}")
    
    # Register a listener to see state changes
    def on_state_change(old_state, new_state):
        print(f"State transition: {old_state} -> {new_state}")
    
    state_manager.add_state_listener(on_state_change)
    
    # Simulate state progression
    print("\nSimulating state transitions:")
    
    # Boot to idle
    if state_manager.transition_to(UIState.IDLE, "System booted successfully"):
        print(f"  -> Now in {state_manager.current_state} state")
    
    # Wait for engagement
    if state_manager.transition_to(UIState.AWAITING_ENGAGEMENT, "User requested engagement"):
        print(f"  -> Now in {state_manager.current_state} state")
    
    # Engage the system
    state_manager.set_context('safety_ok', True)
    state_manager.set_context('sensors_ok', True)
    if state_manager.transition_to(UIState.ENGAGED, "System is now engaged"):
        print(f"  -> Now in {state_manager.current_state} state")
    
    # Simulate emergency
    if state_manager.transition_to(UIState.EMERGENCY, "Emergency condition detected"):
        print(f"  -> Now in {state_manager.current_state} state")
    
    # Return to idle after emergency
    if state_manager.transition_to(UIState.IDLE, "Emergency resolved, back to idle"):
        print(f"  -> Now in {state_manager.current_state} state")
    
    # Show transition history
    print(f"\nRecent transitions:")
    for trans in state_manager.get_recent_transitions(10):
        status = "✓" if trans.valid else "✗"
        print(f"  {status} {trans.from_state} -> {trans.to_state} ({trans.reason})")
    
    # Show state durations
    print(f"\nState durations:")
    for state in UIState:
        duration = state_manager.get_state_duration(state)
        if duration > 0:
            print(f"  {state.value}: {duration:.2f}s")


if __name__ == "__main__":
    main()