#!/usr/bin/env python3

import os
import sys
import time
import math
import argparse
# import setup_path
from abc import ABC, abstractmethod
from datetime import datetime

# Add parent directory to path for AirSim imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import airsim

class GenericMission(ABC):
    """
    Abstract base class for all drone missions using Template Method pattern.
    Provides common infrastructure for takeoff, landing, tracking, and analytics.
    """
    
    def __init__(self, altitude=20, speed=5):
        """Initialize the mission with basic parameters."""
        self.altitude = altitude
        self.speed = speed
        
        # Mission tracking
        self.start_time = None
        self.end_time = None
        self.phase_times = {}
        self.total_distance = 0.0
        self.previous_position = None
        self.errors = []
        self.mission_success = False
        self.home_position = None
        
        # Collision detection and safety
        self.collision_count = 0
        self.max_collisions = 5
        self.last_collision_time = 0
        self.collision_threshold = 1.0  # Minimum time between collision detections
        self.emergency_landing_triggered = False
        self.takeoff_time = None  # Track when takeoff completed
        self.takeoff_grace_period = 5.0  # Ignore collisions for 5 seconds after takeoff
        self.mission_paused = False  # Track if mission is paused for recovery
        self.collision_detection_enabled = True  # Can be temporarily disabled
        
        # Collision filtering configuration
        self.ignore_landscape_collisions = True  # Ignore terrain/landscape collisions
        self.ignored_collision_objects = []  # Custom objects to ignore (can be set by subclasses)
        
        # Custom metrics (to be populated by subclasses)
        self.custom_metrics = {}
        
        # AirSim client (initialized in run_mission)
        self.client = None
        
    def run_mission(self):
        """
        Main mission execution using Template Method pattern.
        This method defines the algorithm structure while delegating
        specific steps to subclass implementations.
        """
        print("=" * 60)
        print("AUTONOMOUS DRONE MISSION SYSTEM")
        print("=" * 60)
        
        try:
            # Phase 1: Initialize
            self._initialize_mission()
            
            # Phase 2: Display mission parameters
            self.print_mission_parameters()
            
            # Phase 3: Connect and setup
            self._connect_and_setup()
            
            # Phase 4: Execute mission workflow
            self._execute_mission_workflow()
            
            # Phase 5: Generate final report
            self._generate_mission_report()
            
        except KeyboardInterrupt:
            print("\nMission interrupted by user!")
            self._emergency_landing()
        except Exception as e:
            print(f"\nMission failed with error: {str(e)}")
            self.errors.append(f"Critical Error: {str(e)}")
            self._emergency_landing()
        finally:
            if self.client:
                self.client.armDisarm(False)
                self.client.enableApiControl(False)
    
    def _initialize_mission(self):
        """Initialize mission tracking variables."""
        self.start_time = time.time()
        self.phase_times = {
            'initialization': time.time(),
            'takeoff': None,
            'climb': None,
            'mission_execution': None,
            'return': None,
            'landing': None
        }
        print("Mission initialized")
    
    def _connect_and_setup(self):
        """Connect to AirSim and perform initial setup."""
        print("\nConnecting to AirSim...")
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        print("Connected to AirSim")
        
        # Enable API control and arm
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        
        # Record home position
        self.home_position = self.client.getMultirotorState().kinematics_estimated.position
        self.previous_position = self.home_position
        
        print(f"Home position: ({self.home_position.x_val:.1f}, {self.home_position.y_val:.1f}, {self.home_position.z_val:.1f})")
    
    def _execute_mission_workflow(self):
        """Execute the main mission workflow."""
        # Phase 1: Takeoff
        self._takeoff()
        
        # Phase 2: Climb to target altitude
        target_z = -abs(self.altitude)  # Negative for up in NED
        self._climb_to_altitude(target_z)
        
        # Phase 3: Execute mission-specific logic
        self.phase_times['mission_execution'] = time.time()
        print(f"\nStarting mission execution...")
        self.execute_mission_logic(target_z)
        
        # Phase 4: Return to home
        self._return_to_home()
        
        # Phase 5: Land
        self._land()
        
        self.mission_success = True
        print("\nMission completed successfully!")
    
    def _takeoff(self):
        """Perform takeoff procedure."""
        self.phase_times['takeoff'] = time.time()
        print("\nTaking off...")
        
        try:
            self.client.takeoffAsync().join()
            time.sleep(2)  # Allow stabilization
            self.takeoff_time = time.time()  # Record takeoff completion for collision grace period
            print("Takeoff completed")
            print(f"Collision detection grace period: {self.takeoff_grace_period}s (ignoring ground contact)")
        except Exception as e:
            error_msg = f"Takeoff failed: {str(e)}"
            self.errors.append(error_msg)
            raise Exception(error_msg)
    
    def _climb_to_altitude(self, target_z):
        """Climb to the specified altitude."""
        self.phase_times['climb'] = time.time()
        print(f"Climbing to {abs(target_z)}m altitude...")
        
        try:
            self.client.moveToZAsync(target_z, self.speed).join()
            time.sleep(1)  # Allow stabilization
            
            # Verify altitude
            current_position = self.client.getMultirotorState().kinematics_estimated.position
            actual_altitude = abs(current_position.z_val)
            print(f"Reached altitude: {actual_altitude:.1f}m")
            
        except Exception as e:
            error_msg = f"Climb failed: {str(e)}"
            self.errors.append(error_msg)
            raise Exception(error_msg)
    
    def _return_to_home(self):
        """Return to home position."""
        self.phase_times['return'] = time.time()
        print("\nReturning to home...")
        
        try:
            self.client.moveToPositionAsync(
                self.home_position.x_val,
                self.home_position.y_val,
                -abs(self.altitude),  # Maintain altitude
                self.speed
            ).join()
            print("Returned to home position")
            
        except Exception as e:
            error_msg = f"Return to home failed: {str(e)}"
            self.errors.append(error_msg)
            print(f"{error_msg}")
    
    def _land(self):
        """Perform landing procedure."""
        self.phase_times['landing'] = time.time()
        print("\n Landing...")
        
        try:
            self.client.landAsync().join()
            time.sleep(2)  # Allow landing completion
            print("Landing completed")
            
        except Exception as e:
            error_msg = f"Landing failed: {str(e)}"
            self.errors.append(error_msg)
            print(f"{error_msg}")
    
    def _emergency_landing(self):
        """Perform emergency landing procedure."""
        print("\nEMERGENCY LANDING INITIATED")
        
        if self.client:
            try:
                # Try normal landing first
                self.client.landAsync().join()
                print("Emergency landing completed")
            except:
                try:
                    # If normal landing fails, try hover and manual descent
                    self.client.hoverAsync().join()
                    self.client.moveByVelocityZAsync(0, 0, 0, 2).join()  # Slow descent
                    print("Emergency hover and descent completed")
                except:
                    print("Emergency landing failed - manual intervention required")
    
    def update_distance_tracking(self):
        """Update total distance traveled and check for collisions."""
        if self.previous_position is None:
            return
            
        try:
            current_position = self.client.getMultirotorState().kinematics_estimated.position
            
            # Calculate 3D distance
            dx = current_position.x_val - self.previous_position.x_val
            dy = current_position.y_val - self.previous_position.y_val
            dz = current_position.z_val - self.previous_position.z_val
            
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            self.total_distance += distance
            
            self.previous_position = current_position
            
            # Check for collisions during tracking
            self._check_collision()
            
        except Exception as e:
            self.errors.append(f"Distance tracking error: {str(e)}")
    
    def _check_collision(self):
        """Check for collision and handle emergency landing if threshold exceeded."""
        # Skip collision detection if temporarily disabled
        if not self.collision_detection_enabled:
            return
            
        try:
            collision_info = self.client.simGetCollisionInfo()
            current_time = time.time()
            
            # Check if we're in the takeoff grace period
            if self.takeoff_time and (current_time - self.takeoff_time) < self.takeoff_grace_period:
                # During grace period, only check for collisions but don't count ground contact
                if collision_info.has_collided:
                    # Check if this is likely ground contact (z position close to ground)
                    current_pos = self.client.getMultirotorState().kinematics_estimated.position
                    if abs(current_pos.z_val) < 2.0:  # Within 2 meters of ground
                        return  # Ignore ground contact during grace period
            
            # Check if a collision occurred
            if collision_info.has_collided:
                # Get collision details
                collision_point = collision_info.impact_point
                object_name = collision_info.object_name if hasattr(collision_info, 'object_name') else "Unknown"
                
                # Filter out landscape and terrain collisions
                if self.ignore_landscape_collisions and self._is_landscape_collision(object_name):
                    # Log landscape collision for debugging but don't count it
                    print(f"Landscape contact detected (ignored): {object_name}")
                    return  # Don't count as collision
                
                # Filter out custom ignored objects
                if self._is_ignored_object(object_name):
                    # Log ignored collision for debugging but don't count it
                    print(f"Ignored object collision detected: {object_name}")
                    return  # Don't count as collision
                
                # Avoid counting the same collision multiple times
                if current_time - self.last_collision_time > self.collision_threshold:
                    self.collision_count += 1
                    self.last_collision_time = current_time
                    
                    print(f"COLLISION DETECTED #{self.collision_count}")
                    print(f"   Object: {object_name}")
                    print(f"   Impact Point: ({collision_point.x_val:.1f}, {collision_point.y_val:.1f}, {collision_point.z_val:.1f})")
                    
                    # Log collision
                    self.errors.append(f"Collision #{self.collision_count} with {object_name} at ({collision_point.x_val:.1f}, {collision_point.y_val:.1f}, {collision_point.z_val:.1f})")
                    
                    # IMMEDIATELY pause mission for all collision responses
                    self.mission_paused = True
                    
                    # Stop all current movement immediately
                    try:
                        self.client.cancelLastTask()
                        self.client.hoverAsync().join()
                    except:
                        pass
                    
                    # Check if collision limit exceeded
                    if self.collision_count >= self.max_collisions:
                        print(f"COLLISION LIMIT EXCEEDED! ({self.collision_count}/{self.max_collisions})")
                        print("INITIATING EMERGENCY LANDING!")
                        self._emergency_collision_landing()
                        raise Exception(f"Mission aborted: {self.collision_count} collisions exceeded safety limit")
                    else:
                        remaining = self.max_collisions - self.collision_count
                        print(f"  Warning: {remaining} collisions remaining before emergency landing")
                        
                        # Attempt recovery
                        recovery_success = self._attempt_collision_recovery()
                        
                        if not recovery_success:
                            print(" COLLISION RECOVERY FAILED - EMERGENCY LANDING!")
                            self._emergency_collision_landing()
                            raise Exception(f"Mission aborted: Collision recovery failed after {self.collision_count} collisions")
                        
                        # Only resume if recovery was successful
                        self.mission_paused = False
            
        except Exception as e:
            if "Mission aborted" not in str(e):
                self.errors.append(f"Collision detection error: {str(e)}")
            else:
                raise  # Re-raise mission abort exceptions
    
    def _attempt_collision_recovery(self):
        """Attempt to recover from collision by backing away and hovering."""
        try:
            print(" Attempting collision recovery...")
            
            # Stop ALL current movement immediately
            self.client.cancelLastTask()
            time.sleep(0.5)  # Allow cancellation to take effect
            
            # Emergency hover to stabilize
            self.client.hoverAsync().join()
            time.sleep(1)
            
            # Get current position
            current_pos = self.client.getMultirotorState().kinematics_estimated.position
            
            # Calculate safe recovery position
            # Move back and up significantly to clear obstacles
            recovery_x = current_pos.x_val - 5  # Move back 5 meters
            recovery_y = current_pos.y_val - 2  # Move sideways 2 meters  
            recovery_z = current_pos.z_val - 5  # Move up 5 meters (higher safety margin)
            
            print(f" Moving to recovery position ({recovery_x:.1f}, {recovery_y:.1f}, {recovery_z:.1f})...")
            
            # Use slower, more careful movement for recovery
            try:
                self.client.moveToPositionAsync(recovery_x, recovery_y, recovery_z, 1.5).join()
                time.sleep(2)  # Extended stabilization time
                
                # Verify we're in a safe position (check for immediate collision)
                collision_check = self.client.simGetCollisionInfo()
                if collision_check.has_collided:
                    print("  Still in collision after recovery attempt")
                    return False
                
                print("Collision recovery completed")
                return True
                
            except Exception as recovery_error:
                print(f"Recovery movement failed: {str(recovery_error)}")
                return False
            
        except Exception as e:
            print(f"Collision recovery failed: {str(e)}")
            self.errors.append(f"Collision recovery failed: {str(e)}")
            
            # Last resort - try emergency hover
            try:
                self.client.hoverAsync().join()
                print("  Emergency hover activated")
                return False  # Recovery failed
            except:
                print("Critical recovery failure")
                return False
    
    def _emergency_collision_landing(self):
        """Perform emergency landing due to collision limit exceeded."""
        self.emergency_landing_triggered = True
        self.mission_paused = True  # Ensure mission is completely paused
        
        print("\n" + "="*50)
        print(" EMERGENCY LANDING - COLLISION LIMIT EXCEEDED")
        print("="*50)
        
        try:
            # IMMEDIATELY stop all movement and tasks
            self.client.cancelLastTask()
            time.sleep(1)  # Allow cancellation to fully take effect
            
            # Get current position
            current_pos = self.client.getMultirotorState().kinematics_estimated.position
            print(f" Current position: ({current_pos.x_val:.1f}, {current_pos.y_val:.1f}, {current_pos.z_val:.1f})")
            
            # Emergency stabilization
            print(" Emergency stabilization...")
            self.client.hoverAsync().join()
            time.sleep(3)  # Extended stabilization time
            
            # Try to clear obstacles by moving up and away from collision area
            print(" Clearing collision area...")
            try:
                # Move up and back to clear obstacles
                safe_altitude = current_pos.z_val - 10  # Move up 10 meters
                safe_x = current_pos.x_val - 8  # Move back 8 meters
                safe_y = current_pos.y_val - 3  # Move sideways 3 meters
                
                self.client.moveToPositionAsync(safe_x, safe_y, safe_altitude, 1).join()
                time.sleep(2)
                
                # Final hover before landing
                self.client.hoverAsync().join()
                time.sleep(2)
                
            except Exception as clear_error:
                print(f"  Could not clear obstacles: {str(clear_error)}")
                print("Proceeding with emergency landing at current position...")
            
            # Attempt controlled emergency descent
            print(" Beginning emergency descent...")
            try:
                self.client.landAsync().join()
                time.sleep(5)  # Extended landing time
                print("Emergency landing completed")
            except Exception as land_error:
                print(f"  Controlled landing failed: {str(land_error)}")
                # Try manual descent if landing fails
                try:
                    print(" Attempting manual descent...")
                    current_pos = self.client.getMultirotorState().kinematics_estimated.position
                    ground_level = current_pos.z_val + abs(current_pos.z_val) + 2  # Near ground level
                    self.client.moveToZAsync(ground_level, 0.5).join()  # Very slow descent
                    time.sleep(3)
                    print("Manual emergency descent completed")
                except:
                    print("Manual descent failed - hovering at current position")
            
        except Exception as e:
            print(f"Emergency landing failed: {str(e)}")
            # Absolute last resort - emergency hover
            try:
                self.client.hoverAsync().join()
                print("  EMERGENCY HOVER ACTIVATED")
                print(" MANUAL INTERVENTION REQUIRED")
                print("Drone is hovering in place - use manual controls to land safely")
            except:
                print("CRITICAL SYSTEM FAILURE")
                print(" IMMEDIATE MANUAL INTERVENTION REQUIRED")
                print("All automated systems failed - take manual control immediately")
    
    def _is_landscape_collision(self, object_name):
        """Check if collision is with landscape/terrain that should be ignored."""
        if not object_name:
            return False
        
        # Convert to lowercase for case-insensitive comparison
        obj_name_lower = object_name.lower()
        
        # Common landscape/terrain object identifiers
        landscape_keywords = [
            'landscape',
            'terrain',
            'ground',
            'heightfield',
            'streaming',
            'worldpartition',
            'landscape_streaming',
            'terrainproxy',
            'heightfieldcollision',
            'landscapeheightfieldcollisioncomponent',
            'landscapemeshcollisioncomponent',
            'hlod',  # Hierarchical Level of Detail
            'worldpartitionstreamingproxy',
            'landscapeproxy',
            'foliage',  # Sometimes foliage is attached to landscape
            'grass',
            'worldpartitionruntimehash'
        ]
        
        # Check if object name contains any landscape keywords
        for keyword in landscape_keywords:
            if keyword in obj_name_lower:
                return True
        
        # Additional check for streaming assets pattern (e.g., "StreamingActor_123")
        if 'streaming' in obj_name_lower and ('actor' in obj_name_lower or 'proxy' in obj_name_lower):
            return True
        
        return False
    
    def _is_ignored_object(self, object_name):
        """Check if collision is with a custom ignored object."""
        if not object_name or not self.ignored_collision_objects:
            return False
        
        obj_name_lower = object_name.lower()
        
        # Check if object name contains any custom ignored keywords
        for ignored_object in self.ignored_collision_objects:
            if ignored_object.lower() in obj_name_lower:
                return True
        
        return False
    
    def add_ignored_collision_object(self, object_name):
        """Add an object name/pattern to ignore in collision detection."""
        if object_name not in self.ignored_collision_objects:
            self.ignored_collision_objects.append(object_name)
            print(f"Added '{object_name}' to ignored collision objects")
    
    def remove_ignored_collision_object(self, object_name):
        """Remove an object name/pattern from ignored collision detection."""
        if object_name in self.ignored_collision_objects:
            self.ignored_collision_objects.remove(object_name)
            print(f"Removed '{object_name}' from ignored collision objects")
    
    def set_landscape_collision_detection(self, enabled):
        """Enable or disable landscape collision detection."""
        self.ignore_landscape_collisions = not enabled
        status = "enabled" if enabled else "disabled"
        print(f" Landscape collision detection {status}")
    
    def disable_collision_detection(self):
        """Temporarily disable collision detection (e.g., during precise maneuvers)."""
        self.collision_detection_enabled = False
        print(f"Collision detection disabled")
    
    def enable_collision_detection(self):
        """Re-enable collision detection."""
        self.collision_detection_enabled = True
        print(f"Collision detection enabled")
    
    def _monitor_collision_during_mission(self):
        """Continuous collision monitoring during mission execution."""
        if self.emergency_landing_triggered:
            raise Exception("Mission aborted due to collision emergency landing")
        
        self._check_collision()
    
    def take_photo(self, photo_dir="photos", filename_prefix="photo"):
        """Take a photo and save it to the specified directory."""
        # Don't take photos if emergency landing is triggered
        if self.emergency_landing_triggered:
            return None
            
        try:
            # Create photos directory if it doesn't exist
            os.makedirs(photo_dir, exist_ok=True)
            
            # Generate timestamp-based filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Include milliseconds
            filename = f"{filename_prefix}_{timestamp}.png"
            filepath = os.path.join(photo_dir, filename)
            
            # Take photo
            responses = self.client.simGetImages([
                airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)
            ])
            
            if responses:
                response = responses[0]
                if response.pixels_as_float:
                    airsim.write_pfm(filepath, airsim.get_pfm_array(response))
                else:
                    airsim.write_file(filepath, response.image_data_uint8)
                
                print(f"Photo saved: {filename}")
                return filepath
            else:
                error_msg = "Failed to capture image"
                self.errors.append(error_msg)
                return None
                
        except Exception as e:
            error_msg = f"Photo capture failed: {str(e)}"
            self.errors.append(error_msg)
            print(f"  {error_msg}")
            return None
    
    def _generate_mission_report(self):
        """Generate comprehensive post-mission analysis report."""
        self.end_time = time.time()
        total_duration = self.end_time - self.start_time
        
        print("\n" + "=" * 60)
        print("MISSION ANALYSIS REPORT")
        print("=" * 60)
        
        # Mission Overview
        print("\nMISSION OVERVIEW")
        print(f"Status: {'SUCCESS' if self.mission_success else 'FAILED'}")
        print(f"Total Duration: {total_duration:.1f} seconds ({total_duration/60:.1f} minutes)")
        print(f"Mission Type: {self.__class__.__name__}")
        
        # Performance Metrics
        planned_distance = self.get_planned_distance()
        efficiency = (planned_distance / self.total_distance * 100) if self.total_distance > 0 else 0
        avg_speed = self.total_distance / total_duration if total_duration > 0 else 0
        
        print(f"\nPERFORMANCE METRICS")
        print(f"Distance Planned: {planned_distance:.1f}m")
        print(f"Distance Actual: {self.total_distance:.1f}m")
        print(f"Flight Efficiency: {efficiency:.1f}%")
        print(f"Average Speed: {avg_speed:.1f}m/s")
        
        # Safety Metrics
        print(f"\nSAFETY METRICS")
        print(f"Collisions Detected: {self.collision_count}/{self.max_collisions}")
        if self.collision_count > 0:
            print(f"Collision Status: {' CRITICAL' if self.collision_count >= self.max_collisions else '  WARNING'}")
        else:
            print(f"Collision Status: SAFE")
        print(f"Emergency Landing: {' YES' if self.emergency_landing_triggered else 'NO'}")
        
        # Phase Breakdown
        print(f"\nPHASE BREAKDOWN")
        phases = ['takeoff', 'climb', 'mission_execution', 'return', 'landing']
        for i, phase in enumerate(phases):
            if self.phase_times[phase] is not None:
                if i + 1 < len(phases) and phases[i + 1] in self.phase_times and self.phase_times[phases[i + 1]] is not None:
                    duration = self.phase_times[phases[i + 1]] - self.phase_times[phase]
                else:
                    duration = self.end_time - self.phase_times[phase]
                print(f"{phase.replace('_', ' ').title()}: {duration:.1f}s")
        
        # Mission-Specific Metrics
        mission_metrics = self.get_mission_specific_metrics()
        if mission_metrics:
            print(f"\nMISSION-SPECIFIC METRICS")
            for key, value in mission_metrics.items():
                print(f"{key}: {value}")
        
        # Quality Assessment
        quality_score = self._calculate_quality_score(efficiency)
        grade = self._get_quality_grade(quality_score)
        print(f"\n QUALITY ASSESSMENT")
        print(f"Mission Score: {quality_score}/100")
        print(f"Grade: {grade}")
        
        # Error Report
        if self.errors:
            print(f"\n  ERROR REPORT")
            for i, error in enumerate(self.errors, 1):
                print(f"{i}. {error}")
        else:
            print(f"\nNO ERRORS RECORDED")
        
        # Recommendations
        recommendations = self._generate_recommendations(efficiency, quality_score)
        if recommendations:
            print(f"\nRECOMMENDATIONS")
            for i, rec in enumerate(recommendations, 1):
                print(f"{i}. {rec}")
    
    def _calculate_quality_score(self, efficiency):
        """Calculate mission quality score (0-100)."""
        score = 0
        
        # Mission success (40 points)
        if self.mission_success and not self.emergency_landing_triggered:
            score += 40
        elif self.mission_success and self.emergency_landing_triggered:
            score += 20  # Partial credit if mission completed but had emergency landing
        
        # Flight efficiency (30 points)
        if efficiency >= 90:
            score += 30
        elif efficiency >= 80:
            score += 25
        elif efficiency >= 70:
            score += 20
        elif efficiency >= 60:
            score += 15
        elif efficiency >= 50:
            score += 10
        
        # Collision penalties (severe)
        if self.collision_count > 0:
            collision_penalty = self.collision_count * 15  # 15 points per collision
            score -= collision_penalty
            print(f"   Collision Penalty: -{collision_penalty} points ({self.collision_count} collisions)")
        
        # Emergency landing penalty (severe)
        if self.emergency_landing_triggered:
            score -= 25
            print(f"   Emergency Landing Penalty: -25 points")
        
        # Error penalty (-10 points per error)
        if len(self.errors) > 0:
            error_penalty = len(self.errors) * 10
            score -= error_penalty
            print(f"   Error Penalty: -{error_penalty} points ({len(self.errors)} errors)")
        
        # Safe landing bonus (30 points) - only if no emergency landing
        if not any("landing failed" in error.lower() for error in self.errors) and not self.emergency_landing_triggered:
            score += 30
        
        return max(0, min(100, score))  # Clamp between 0-100
    
    def _get_quality_grade(self, score):
        """Convert quality score to letter grade."""
        if score >= 95:
            return "A+ (Exceptional)"
        elif score >= 90:
            return "A (Excellent)"
        elif score >= 85:
            return "A- (Very Good)"
        elif score >= 80:
            return "B+ (Good)"
        elif score >= 75:
            return "B (Satisfactory)"
        elif score >= 70:
            return "B- (Acceptable)"
        elif score >= 65:
            return "C+ (Below Average)"
        elif score >= 60:
            return "C (Poor)"
        elif score >= 55:
            return "C- (Very Poor)"
        else:
            return "D (Unacceptable)"
    
    def _generate_recommendations(self, efficiency, quality_score):
        """Generate actionable recommendations based on mission performance."""
        recommendations = []
        
        # Collision-related recommendations (highest priority)
        if self.collision_count > 0:
            recommendations.append(f" CRITICAL: {self.collision_count} collision(s) detected - review flight path and obstacle avoidance")
        
        if self.emergency_landing_triggered:
            recommendations.append(" EMERGENCY: Mission aborted due to collision limit - investigate environment and safety protocols")
        
        if self.collision_count >= 3:
            recommendations.append("  Consider increasing flight altitude or using more conservative flight paths")
        
        # Performance recommendations
        if efficiency < 70:
            recommendations.append("Consider optimizing flight path for better efficiency")
        
        if len(self.errors) > 0:
            recommendations.append("Review error log and implement preventive measures")
        
        if quality_score < 80:
            recommendations.append("Review mission parameters and flight procedures")
        
        if self.total_distance == 0:
            recommendations.append("Verify distance tracking system is working properly")
        
        # Safety recommendations
        if self.collision_count > 0 and not self.emergency_landing_triggered:
            recommendations.append("Consider implementing more aggressive collision avoidance measures")
        
        return recommendations
    
    # Abstract methods that must be implemented by subclasses
    
    @abstractmethod
    def print_mission_parameters(self):
        """Print mission-specific parameters. Must be implemented by subclasses."""
        pass
    
    @abstractmethod
    def execute_mission_logic(self, target_z):
        """Execute the core mission logic. Must be implemented by subclasses."""
        pass
    
    @abstractmethod
    def get_planned_distance(self):
        """Return the planned mission distance for efficiency calculation."""
        pass
    
    @abstractmethod
    def get_mission_specific_metrics(self):
        """Return a dictionary of mission-specific metrics for the report."""
        pass


def create_standard_argument_parser():
    """Create a standard argument parser with common mission parameters."""
    parser = argparse.ArgumentParser(description="Autonomous Drone Mission")
    parser.add_argument('--altitude', type=float, default=20, 
                       help='Flight altitude in meters (default: 20)')
    parser.add_argument('--speed', type=float, default=5,
                       help='Flight speed in m/s (default: 5)')
    parser.add_argument('--preview', action='store_true',
                       help='Preview mission parameters without flying')
    parser.add_argument('--max_collisions', type=int, default=5,
                       help='Maximum collisions before emergency landing (default: 5)')
    parser.add_argument('--include_landscape_collisions', action='store_true',
                       help='Include landscape/terrain collisions in safety system (default: False)')
    parser.add_argument('--ignore_objects', type=str, nargs='*', default=[],
                       help='Object names/patterns to ignore in collision detection')
    return parser


if __name__ == "__main__":
    print("GenericMission is an abstract base class and cannot be run directly.")
    print("Please run a specific mission type like BoxMission or SpiralSearchMission.") 