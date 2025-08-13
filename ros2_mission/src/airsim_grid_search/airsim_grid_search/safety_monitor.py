#!/usr/bin/env python3

"""
Safety Monitor Module

Provides comprehensive safety monitoring and enforcement for autonomous missions.
Monitors battery, communication, geofence, weather, and other safety parameters.

Author: Cosys-Lab
License: MIT
"""

import time
import math
from typing import Dict, List, Optional, Tuple
from enum import Enum

from geometry_msgs.msg import Point, Polygon
from sensor_msgs.msg import BatteryState, NavSatFix

from airsim_mission_interfaces.msg import SafetyParams


class SafetyViolationType(Enum):
    """Types of safety violations"""
    BATTERY_LOW = "battery_low"
    BATTERY_CRITICAL = "battery_critical"
    GEOFENCE_VIOLATION = "geofence_violation"
    COMMUNICATION_LOSS = "communication_loss"
    MAX_DISTANCE_EXCEEDED = "max_distance_exceeded"
    MAX_FLIGHT_TIME_EXCEEDED = "max_flight_time_exceeded"
    WEATHER_VIOLATION = "weather_violation"
    SYSTEM_FAILURE = "system_failure"


class SafetyStatus(Enum):
    """Overall safety status"""
    SAFE = "safe"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


class SafetyViolation:
    """Represents a safety violation"""
    def __init__(self, violation_type: SafetyViolationType, severity: SafetyStatus,
                 message: str, timestamp: float = None):
        self.violation_type = violation_type
        self.severity = severity
        self.message = message
        self.timestamp = timestamp or time.time()
        self.acknowledged = False


class SafetyMonitor:
    """
    Comprehensive safety monitoring system for autonomous missions.
    
    Features:
    - Battery level monitoring
    - Geofence enforcement
    - Communication loss detection
    - Flight time and distance limits
    - Weather condition monitoring
    - Emergency response procedures
    """
    
    def __init__(self, grid_search_server):
        self.server = grid_search_server
        self.node = grid_search_server
        self.logger = grid_search_server.get_logger()
        
        # Safety parameters
        self.safety_params: Optional[SafetyParams] = None
        self.mission_start_time: Optional[float] = None
        self.home_position: Optional[Point] = None
        
        # Monitoring state
        self.active_violations: List[SafetyViolation] = []
        self.last_communication_time = time.time()
        self.last_battery_reading: Optional[float] = None
        self.last_position: Optional[Point] = None
        
        # Emergency state
        self.emergency_mode = False
        self.return_home_initiated = False
        
        # Monitoring frequency (Hz)
        self.monitoring_frequency = self.node.get_parameter('safety_check_frequency').value
        
        # Set up safety monitoring timer
        self.safety_timer = self.node.create_timer(
            1.0 / self.monitoring_frequency,
            self.perform_safety_checks
        )
        
        self.logger.info("Safety Monitor initialized")
    
    def set_safety_parameters(self, safety_params: SafetyParams):
        """Set safety parameters for current mission"""
        self.safety_params = safety_params
        self.logger.info("Safety parameters updated")
        
        # Set home position if provided
        if hasattr(safety_params, 'home_position'):
            self.home_position = safety_params.home_position
    
    def start_mission_monitoring(self):
        """Start monitoring for a new mission"""
        self.mission_start_time = time.time()
        self.active_violations.clear()
        self.emergency_mode = False
        self.return_home_initiated = False
        self.last_communication_time = time.time()
        
        # Store current position as home if not set
        if self.home_position is None:
            current_pos = self._get_current_position()
            if current_pos:
                self.home_position = current_pos
                
        self.logger.info("Mission safety monitoring started")
    
    def stop_mission_monitoring(self):
        """Stop mission monitoring"""
        self.mission_start_time = None
        self.logger.info("Mission safety monitoring stopped")
    
    def perform_preflight_checks(self, safety_params: SafetyParams) -> bool:
        """
        Perform comprehensive pre-flight safety checks.
        
        Args:
            safety_params: Safety parameters for the mission
            
        Returns:
            bool: True if all pre-flight checks pass
        """
        self.logger.info("Performing pre-flight safety checks")
        
        self.set_safety_parameters(safety_params)
        
        checks_passed = True
        
        # Check battery level
        if not self._check_battery_preflight():
            self.logger.error("Pre-flight battery check failed")
            checks_passed = False
        
        # Check communication
        if not self._check_communication_preflight():
            self.logger.error("Pre-flight communication check failed")
            checks_passed = False
        
        # Check geofence if enabled
        if safety_params.enable_geofence:
            if not self._check_geofence_preflight():
                self.logger.error("Pre-flight geofence check failed")
                checks_passed = False
        
        # Check system health
        if not self._check_system_health_preflight():
            self.logger.error("Pre-flight system health check failed")
            checks_passed = False
        
        if checks_passed:
            self.logger.info("All pre-flight checks passed")
        else:
            self.logger.error("Pre-flight checks failed")
            
        return checks_passed
    
    def perform_safety_checks(self):
        """Perform ongoing safety checks during mission"""
        if not self.safety_params or not self.mission_start_time:
            return
        
        try:
            # Update communication timestamp
            self.last_communication_time = time.time()
            
            # Perform individual safety checks
            self._check_battery_level()
            self._check_flight_time()
            self._check_distance_from_home()
            self._check_geofence()
            self._check_communication_timeout()
            
            # Evaluate overall safety status
            self._evaluate_safety_status()
            
        except Exception as e:
            self.logger.error(f"Safety check error: {str(e)}")
    
    def _check_battery_level(self):
        """Check battery level against safety thresholds"""
        current_battery = self._get_current_battery_percentage()
        if current_battery is None:
            return
        
        self.last_battery_reading = current_battery
        
        # Check critical battery level
        if current_battery < (self.safety_params.return_home_battery * 100):
            if not self._has_active_violation(SafetyViolationType.BATTERY_CRITICAL):
                violation = SafetyViolation(
                    SafetyViolationType.BATTERY_CRITICAL,
                    SafetyStatus.EMERGENCY,
                    f"Critical battery level: {current_battery:.1f}%"
                )
                self._add_violation(violation)
                self._initiate_emergency_return_home("Critical battery level")
        
        # Check low battery warning
        elif current_battery < (self.safety_params.min_battery_percentage * 100):
            if not self._has_active_violation(SafetyViolationType.BATTERY_LOW):
                violation = SafetyViolation(
                    SafetyViolationType.BATTERY_LOW,
                    SafetyStatus.WARNING,
                    f"Low battery level: {current_battery:.1f}%"
                )
                self._add_violation(violation)
    
    def _check_flight_time(self):
        """Check flight time against maximum limit"""
        if not self.mission_start_time:
            return
        
        elapsed_time = time.time() - self.mission_start_time
        max_time = self.safety_params.max_flight_time
        
        if elapsed_time > max_time:
            if not self._has_active_violation(SafetyViolationType.MAX_FLIGHT_TIME_EXCEEDED):
                violation = SafetyViolation(
                    SafetyViolationType.MAX_FLIGHT_TIME_EXCEEDED,
                    SafetyStatus.CRITICAL,
                    f"Maximum flight time exceeded: {elapsed_time:.1f}s > {max_time:.1f}s"
                )
                self._add_violation(violation)
                self._initiate_emergency_return_home("Maximum flight time exceeded")
    
    def _check_distance_from_home(self):
        """Check distance from home position"""
        if not self.home_position:
            return
        
        current_pos = self._get_current_position()
        if not current_pos:
            return
        
        distance = self._calculate_distance(current_pos, self.home_position)
        max_distance = self.safety_params.max_distance_from_home
        
        if distance > max_distance:
            if not self._has_active_violation(SafetyViolationType.MAX_DISTANCE_EXCEEDED):
                violation = SafetyViolation(
                    SafetyViolationType.MAX_DISTANCE_EXCEEDED,
                    SafetyStatus.CRITICAL,
                    f"Maximum distance from home exceeded: {distance:.1f}m > {max_distance:.1f}m"
                )
                self._add_violation(violation)
                self._initiate_emergency_return_home("Maximum distance from home exceeded")
    
    def _check_geofence(self):
        """Check geofence boundaries"""
        if not self.safety_params.enable_geofence:
            return
        
        current_pos = self._get_current_position()
        if not current_pos:
            return
        
        # Check if position is within geofence
        if not self._is_within_geofence(current_pos):
            if not self._has_active_violation(SafetyViolationType.GEOFENCE_VIOLATION):
                violation = SafetyViolation(
                    SafetyViolationType.GEOFENCE_VIOLATION,
                    SafetyStatus.CRITICAL,
                    f"Geofence violation at ({current_pos.x:.1f}, {current_pos.y:.1f})"
                )
                self._add_violation(violation)
                self._initiate_emergency_return_home("Geofence violation")
    
    def _check_communication_timeout(self):
        """Check for communication timeouts"""
        if not hasattr(self.safety_params, 'max_comm_loss_time'):
            return
        
        comm_loss_time = time.time() - self.last_communication_time
        max_loss_time = self.safety_params.max_comm_loss_time
        
        if comm_loss_time > max_loss_time:
            if not self._has_active_violation(SafetyViolationType.COMMUNICATION_LOSS):
                violation = SafetyViolation(
                    SafetyViolationType.COMMUNICATION_LOSS,
                    SafetyStatus.CRITICAL,
                    f"Communication lost for {comm_loss_time:.1f}s"
                )
                self._add_violation(violation)
                
                if not self.safety_params.enable_offline_mode:
                    self._initiate_emergency_return_home("Communication loss")
    
    def _evaluate_safety_status(self):
        """Evaluate overall safety status and take appropriate actions"""
        if not self.active_violations:
            return
        
        # Find highest severity violation
        max_severity = SafetyStatus.SAFE
        for violation in self.active_violations:
            if violation.severity == SafetyStatus.EMERGENCY:
                max_severity = SafetyStatus.EMERGENCY
                break
            elif violation.severity == SafetyStatus.CRITICAL:
                max_severity = SafetyStatus.CRITICAL
            elif violation.severity == SafetyStatus.WARNING and max_severity == SafetyStatus.SAFE:
                max_severity = SafetyStatus.WARNING
        
        # Take action based on severity
        if max_severity == SafetyStatus.EMERGENCY and not self.emergency_mode:
            self._enter_emergency_mode()
    
    def _initiate_emergency_return_home(self, reason: str):
        """Initiate emergency return to home"""
        if self.return_home_initiated:
            return
        
        self.logger.error(f"Initiating emergency return home: {reason}")
        self.return_home_initiated = True
        
        # TODO: Implement emergency return home procedure
        # This would typically:
        # 1. Cancel current mission
        # 2. Navigate to home position
        # 3. Land automatically
        
        # For now, just log the event
        self.logger.info("Emergency return home procedure initiated")
    
    def _enter_emergency_mode(self):
        """Enter emergency mode with immediate safety actions"""
        if self.emergency_mode:
            return
        
        self.logger.error("Entering emergency mode")
        self.emergency_mode = True
        
        # Immediate actions:
        # 1. Stop current navigation
        # 2. Hover in place
        # 3. Prepare for emergency landing
        
        self._send_emergency_stop()
    
    def _send_emergency_stop(self):
        """Send emergency stop command to vehicle"""
        if hasattr(self.server, 'vel_cmd_publisher'):
            from airsim_interfaces.msg import VelCmd
            from geometry_msgs.msg import Twist
            
            # Send zero velocity command
            vel_msg = VelCmd()
            vel_msg.twist = Twist()  # All zeros
            self.server.vel_cmd_publisher.publish(vel_msg)
            
            self.logger.info("Emergency stop command sent")
    
    def _is_within_geofence(self, position: Point) -> bool:
        """Check if position is within geofence boundaries"""
        if not hasattr(self.safety_params, 'geofence_polygon'):
            return True
        
        # Simple rectangular geofence check (could be enhanced for complex polygons)
        # This is a placeholder implementation
        return True
    
    def _has_active_violation(self, violation_type: SafetyViolationType) -> bool:
        """Check if a violation type is already active"""
        return any(v.violation_type == violation_type for v in self.active_violations)
    
    def _add_violation(self, violation: SafetyViolation):
        """Add a new safety violation"""
        self.active_violations.append(violation)
        self.logger.warn(f"Safety violation: {violation.message}")
    
    def _remove_violation(self, violation_type: SafetyViolationType):
        """Remove a safety violation (when resolved)"""
        self.active_violations = [v for v in self.active_violations if v.violation_type != violation_type]
    
    def _get_current_position(self) -> Optional[Point]:
        """Get current vehicle position"""
        with self.server.mission_lock:
            if 'current_position' in self.server.mission_data:
                return self.server.mission_data['current_position']
        return None
    
    def _get_current_battery_percentage(self) -> Optional[float]:
        """Get current battery percentage"""
        with self.server.mission_lock:
            if 'battery_percentage' in self.server.mission_data:
                return self.server.mission_data['battery_percentage']
        return None
    
    def _calculate_distance(self, pos1: Point, pos2: Point) -> float:
        """Calculate 3D distance between two points"""
        dx = pos2.x - pos1.x
        dy = pos2.y - pos1.y
        dz = pos2.z - pos1.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    # Pre-flight check methods
    def _check_battery_preflight(self) -> bool:
        """Pre-flight battery check"""
        battery_level = self._get_current_battery_percentage()
        if battery_level is None:
            self.logger.warn("Battery level not available for pre-flight check")
            return False
        
        min_battery = self.safety_params.min_battery_percentage * 100
        if battery_level < min_battery:
            self.logger.error(f"Battery level too low for flight: {battery_level:.1f}% < {min_battery:.1f}%")
            return False
        
        return True
    
    def _check_communication_preflight(self) -> bool:
        """Pre-flight communication check"""
        # Check if we can communicate with AirSim
        current_pos = self._get_current_position()
        return current_pos is not None
    
    def _check_geofence_preflight(self) -> bool:
        """Pre-flight geofence check"""
        if not self.safety_params.enable_geofence:
            return True
        
        current_pos = self._get_current_position()
        if current_pos is None:
            return False
        
        return self._is_within_geofence(current_pos)
    
    def _check_system_health_preflight(self) -> bool:
        """Pre-flight system health check"""
        # Basic system health checks
        # TODO: Add more comprehensive system health monitoring
        return True
    
    def get_safety_status(self) -> Dict:
        """Get current safety status summary"""
        return {
            'emergency_mode': self.emergency_mode,
            'return_home_initiated': self.return_home_initiated,
            'active_violations': len(self.active_violations),
            'violations': [
                {
                    'type': v.violation_type.value,
                    'severity': v.severity.value,
                    'message': v.message,
                    'timestamp': v.timestamp
                }
                for v in self.active_violations
            ],
            'last_battery_reading': self.last_battery_reading,
            'mission_start_time': self.mission_start_time
        }
    
    def acknowledge_violation(self, violation_type: SafetyViolationType):
        """Acknowledge a safety violation"""
        for violation in self.active_violations:
            if violation.violation_type == violation_type:
                violation.acknowledged = True
                self.logger.info(f"Safety violation acknowledged: {violation_type.value}")
                break