#!/usr/bin/env python
"""
Servo Calibration Tool for ORCA Hand
Auto-detects min/max positions for all joints and saves calibration data
"""

import sys
import time
import json
import os
from datetime import datetime
from servo_driver import ServoDriver, SERVO_TO_JOINT_MAP, JOINT_TO_SERVO_MAP
from joint_config import JOINT_PARAMS, CALIBRATION_ORDER, get_joint_params
from scservo_sdk import *

# ============================================================================
# CALIBRATION CONFIGURATION
# ============================================================================

# Calibration settings
STEP_SIZE = 30              # Position increment for limit detection
SEARCH_DELAY = 0.05          # Delay between search steps (seconds)
CENTER_DELAY = 0.6          # Delay at center position
LOG_DIR = "calibration_logs"  # Directory to save calibration logs

# Note: Joint parameters and calibration order are now in joint_config.py

# ============================================================================
# CALIBRATION CLASS
# ============================================================================

class ServoCalibrator:
    """Auto-calibrates servos by detecting physical limits"""
    
    def __init__(self, enable_gui=True):
        """
        Initialize servo driver
        
        Args:
            enable_gui: Enable real-time GUI plotting (default: True)
        """
        self.driver = ServoDriver(enable_gui=enable_gui)
        self.calibration_data = {}
    
    def find_limit(self, joint_name, direction='min', step_size=STEP_SIZE, max_torque=None):
        """
        Find the physical limit of a joint by moving until stopped
        Ignores factory min/max limits and searches dynamically until resistance is detected
        
        Args:
            joint_name: Name of the joint
            direction: 'min' or 'max' to search for minimum or maximum limit
            step_size: Step size for incremental search
            max_torque: Override MAX_TORQUE_THRESHOLD for this operation
            
        Returns:
            dict: {
                'success': bool,
                'limit_position': int or None,
                'stopped_by_safety': bool
            }
        """
        print(f"\n  Searching for {direction.upper()} limit (dynamic detection)...")
        
        # Set max torque threshold
        if max_torque is None:
            max_torque = 150  # Default threshold
        print(f"  Using MAX_TORQUE_THRESHOLD: {max_torque}")
        
        # Get current status
        status = self.driver.get_joint_status(joint_name)
        if not status['success']:
            return {
                'success': False,
                'limit_position': None,
                'stopped_by_safety': False,
                'error': status['error']
            }
        
        servo_id = status['servo_id']
        current_pos = status['position']
        
        # Determine search direction (ignore factory limits)
        if direction == 'min':
            step = -abs(step_size)
            search_bound = 0  # Search towards 0
        else:  # max
            step = abs(step_size)
            search_bound = 6000  # Search towards 4095
        
        print(f"    Starting from position: {current_pos}")
        print(f"    Searching towards: {search_bound}")
        print(f"    Step size: {step}")
        print(f"    Detection: Will stop when servo can't move further or hits resistance")
        
        # Incremental search for the physical limit
        last_successful_position = current_pos
        consecutive_stalls = 0  # Track if servo can't move multiple times
        
        for i in range(300):  # Max 250 steps (to cover full 0-4095 range)
            next_pos = current_pos + step
            
            # Clamp to absolute servo range (0-4095)
            next_pos = max(0, min(4095, next_pos))
            
            # Check if we've reached absolute limits
            if next_pos == 0 or next_pos == 4095:
                print(f"    ⚠️  Reached absolute servo limit at: {next_pos}")
                return {
                    'success': True,
                    'limit_position': last_successful_position,
                    'stopped_by_safety': False
                }
            
            print(f"    Step {i+1}: Moving to {next_pos}...", end='')
            
            # Attempt to move with custom max torque
            result = self.driver.move_joint(joint_name, next_pos, max_torque_threshold=max_torque)
            
            if not result['success']:
                # Safety stop triggered - we found the limit!
                print(f" STOPPED!")
                print(f"    ✓ Physical limit detected at: {last_successful_position}")
                print(f"    Reason: {result['error']}")
                return {
                    'success': True,
                    'limit_position': last_successful_position,
                    'stopped_by_safety': True
                }
            
            # Movement successful
            final_pos = result['final_position']
            movement = abs(final_pos - current_pos)
            print(f" OK (reached {final_pos}, load: {result['final_load']}, moved: {movement})")
            
            # Check if servo actually moved significantly
            if movement < abs(step_size) * 0.3:  # Less than 30% of expected movement
                consecutive_stalls += 1
                print(f"      ⚠️  Limited movement detected ({consecutive_stalls}/3)")
                
                if consecutive_stalls >= 3:
                    # Servo is stalling - we've hit the physical limit
                    print(f"    ✓ Physical limit detected (servo can't move further): {last_successful_position}")
                    return {
                        'success': True,
                        'limit_position': last_successful_position,
                        'stopped_by_safety': False
                    }
            else:
                # Servo moved properly, reset stall counter
                consecutive_stalls = 0
                last_successful_position = final_pos
            
            current_pos = final_pos
            time.sleep(SEARCH_DELAY)
        
        # Max steps reached
        print(f"    ⚠️  Max search steps reached. Last position: {last_successful_position}")
        return {
            'success': True,
            'limit_position': last_successful_position,
            'stopped_by_safety': False
        }
    
    def calibrate_joint(self, joint_name, max_torque=None, return_to_center=True):
        """
        Calibrate a single joint by finding its physical min/max limits
        
        Args:
            joint_name: Name of the joint to calibrate
            max_torque: Override MAX_TORQUE_THRESHOLD for this joint
            return_to_center: Whether to return to center after finding each limit
            
        Returns:
            dict: Calibration result with detected limits
        """
        print("\n" + "="*60)
        print(f"CALIBRATING JOINT: {joint_name}")
        print("="*60)
        
        # Get initial status
        status = self.driver.get_joint_status(joint_name)
        if not status['success']:
            print(f"✗ Error: {status['error']}")
            return None
        
        servo_id = status['servo_id']
        print(f"  Servo ID: {servo_id}")
        print(f"  Current Position: {status['position']}")
        print(f"  Current Load: {status['load']} ({status['load_direction']})")
        print(f"  Factory Limits: {status['limits']['min']} - {status['limits']['max']}")
        
        # Expand servo limits to full range (0-4095) to allow unrestricted search
        print("\n[1/5] Expanding servo limits to full range (0-4095)...")
        expand_result = self.driver.set_joint_limits(joint_name, 0, 4095)
        if not expand_result['success']:
            print(f"  ⚠️  Warning: Could not expand limits: {expand_result['error']}")
            print(f"  Continuing with current limits...")
        else:
            print(f"  ✓ Servo limits expanded to: 0 - 4095")
        time.sleep(0.5)
        
        # Find minimum limit
        print("\n[3/5] Finding MINIMUM limit...")
        min_result = self.find_limit(joint_name, direction='min', max_torque=max_torque)
        if not min_result['success']:
            print(f"  ✗ Failed to find minimum: {min_result.get('error', 'Unknown error')}")
            return None
        detected_min = min_result['limit_position']
        
        # Move to detected min position and set it as middle (2047)
        print(f"\n  Moving to detected MIN position ({detected_min}) and setting as middle (2047)...")
        self.driver.move_joint(joint_name, detected_min)
        time.sleep(CENTER_DELAY)
        print(f"  ✓ Now at MIN position - this is treated as middle reference point")
        
        # Find maximum limit
        print("\n[4/5] Finding MAXIMUM limit...")
        max_result = self.find_limit(joint_name, direction='max', max_torque=max_torque)
        if not max_result['success']:
            print(f"  ✗ Failed to find maximum: {max_result.get('error', 'Unknown error')}")
            return None
        detected_max = max_result['limit_position']
        
        # Return to center of detected range
        if return_to_center:
            detected_center = (detected_min + detected_max) // 2
            print(f"\n  Returning to detected center ({detected_center})...")
            self.driver.move_joint(joint_name, detected_center)
        
        # Store calibration data
        calibration_result = {
            'joint_name': joint_name,
            'servo_id': servo_id,
            'detected_min': detected_min,
            'detected_max': detected_max,
            'detected_range': abs(detected_max - detected_min),
            'factory_min': status['limits']['min'],
            'factory_max': status['limits']['max'],
            'inverted': status['limits']['inverted'],
            'timestamp': datetime.now().isoformat()
        }
        
        self.calibration_data[joint_name] = calibration_result
        
        print("\n" + "="*60)
        print("CALIBRATION COMPLETE")
        print("="*60)
        print(f"  Factory Limits: {status['limits']['min']} - {status['limits']['max']}")
        print(f"  Detected Min: {detected_min}")
        print(f"  Detected Max: {detected_max}")
        print(f"  Range: {abs(detected_max - detected_min)}")
        print("="*60)
        
        # Write detected limits to servo EEPROM
        print("\n[5/5] Writing detected limits to servo EEPROM...")
        write_result = self.driver.set_joint_limits(joint_name, detected_min, detected_max)
        if write_result['success']:
            print(f"  ✓ Limits saved to servo {servo_id}")
            print(f"    Min: {detected_min}")
            print(f"    Max: {detected_max}")
            calibration_result['limits_written'] = True
        else:
            print(f"  ✗ Failed to write limits: {write_result['error']}")
            calibration_result['limits_written'] = False
            calibration_result['write_error'] = write_result['error']
        
        return calibration_result
    
    def calibrate_single_limit(self, joint_name, direction, max_torque=None, max_load_increase=None, return_to_center=True, set_as_middle=False):
        """
        Calibrate a single limit (min or max) for a joint
        
        Args:
            joint_name: Name of the joint to calibrate
            direction: 'min' or 'max'
            max_torque: Override MAX_TORQUE_THRESHOLD for this operation
            max_load_increase: Override MAX_LOAD_INCREASE for this operation
            return_to_center: Whether to return to center after finding limit
            set_as_middle: Whether to set detected position as middle (2047) reference
            
        Returns:
            dict: Calibration result with detected limit
        """
        print(f"\n{'='*60}")
        print(f"Calibrating {joint_name} - {direction.upper()} limit")
        print(f"  Max Torque: {max_torque}, Max Load Increase: {max_load_increase}")
        print(f"  Set as Middle: {set_as_middle}")
        print(f"{'='*60}")
        
        # Get joint status
        status = self.driver.get_joint_status(joint_name)
        if not status['success']:
            print(f"  ✗ Failed to get status: {status['error']}")
            return None
        
        # Expand servo limits to full range first
        expand_result = self.driver.set_joint_limits(joint_name, 0, 4095)
        if not expand_result['success']:
            print(f"  ⚠️  Warning: Could not expand limits: {expand_result['error']}")
        time.sleep(0.3)
        
        # Move to center first
        center = 2048
        print(f"  Moving to center position: {center}")
        result = self.driver.move_joint(joint_name, center, max_torque_threshold=max_torque, max_load_increase=max_load_increase)
        if not result['success']:
            print(f"  ⚠️  Failed to move to center: {result['error']}")
        time.sleep(CENTER_DELAY)
        
        # Find the limit
        print(f"  Finding {direction.upper()} limit...")
        limit_result = self.find_limit(joint_name, direction=direction, max_torque=max_torque)
        
        if not limit_result['success']:
            print(f"  ✗ Failed to find {direction}: {limit_result.get('error', 'Unknown error')}")
            return None
        
        detected_limit = limit_result['limit_position']
        print(f"  ✓ {direction.upper()} limit detected: {detected_limit}")
        
        # Initialize or update calibration data
        if joint_name not in self.calibration_data:
            self.calibration_data[joint_name] = {
                'joint_name': joint_name,
                'servo_id': status['servo_id'],
                'factory_min': status['limits']['min'],
                'factory_max': status['limits']['max'],
                'inverted': status['limits']['inverted'],
            }
        
        # Store the detected limit
        self.calibration_data[joint_name][f'detected_{direction}'] = detected_limit
        
        # Handle set_as_middle option - set detected position as servo's 2047 middle
        if set_as_middle:
            print(f"\n  Setting detected {direction.upper()} position ({detected_limit}) as MIDDLE (2047)...")
            print(f"  This will remap the servo so {detected_limit} becomes position 2047")
            
            # Calculate offset to remap position
            # If detected_limit should be 2047, then:
            # new_min = current_min + (2047 - detected_limit)
            # new_max = current_max + (2047 - detected_limit)
            offset = 2047 - detected_limit
            
            # Get current limits (should be 0-4095 from expansion)
            current_min = 0
            current_max = 4095
            
            # Calculate new limits
            new_min = max(0, min(4095, current_min + offset))
            new_max = max(0, min(4095, current_max + offset))
            
            print(f"  Remapping servo limits: {current_min}-{current_max} → {new_min}-{new_max}")
            print(f"  Offset applied: {offset}")
            
            # Write new limits to servo EEPROM
            write_result = self.driver.set_joint_limits(joint_name, new_min, new_max)
            if write_result['success']:
                print(f"  ✓ Servo remapped! Position {detected_limit} is now at 2047")
                # Move to the new 2047 position (which is the detected_limit in hardware)
                self.driver.move_joint(joint_name, 2047, max_torque_threshold=max_torque, max_load_increase=max_load_increase)
                time.sleep(CENTER_DELAY)
            else:
                print(f"  ✗ Failed to remap servo: {write_result['error']}")
        
        if return_to_center:
            # Calculate center as mean of detected range
            joint_data = self.calibration_data[joint_name]
            if 'detected_min' in joint_data and 'detected_max' in joint_data:
                # Both limits known - return to mean of range
                center_pos = (joint_data['detected_min'] + joint_data['detected_max']) // 2
                print(f"  Returning to center (mean of range: {center_pos})...")
            else:
                # Only one limit known - return to 2048
                center_pos = 2048
                print(f"  Returning to default center ({center_pos}).LIBRAT..")
            
            self.driver.move_joint(joint_name, center_pos, max_torque_threshold=max_torque, max_load_increase=max_load_increase)
            time.sleep(CENTER_DELAY)
        else:
            print(f"  Staying at {direction.upper()} position (no return to center)")
        
        return detected_limit
    
    def calibrate_all_joints(self):
        """
        Calibrate all joints using the configured CALIBRATION_ORDER
        
        The order and parameters are defined in CALIBRATION_ORDER configuration.
        Remaining joints not in CALIBRATION_ORDER will be calibrated with default settings.
        """
        print("\n" + "="*70)
        print("CONFIGURABLE CALIBRATION SEQUENCE - ALL JOINTS")
        print("="*70)
        
        # Track which joints have been calibrated
        calibrated_joints = set()
        
        # Phase 1-3: Follow CALIBRATION_ORDER
        print(f"\nExecuting {len(CALIBRATION_ORDER)} calibration steps from CALIBRATION_ORDER...")
        
        for step_idx, (joint_name, direction, max_torque, max_load_increase, return_to_center, set_as_middle) in enumerate(CALIBRATION_ORDER, 1):
            print(f"\n[Step {step_idx}/{len(CALIBRATION_ORDER)}] {joint_name} - {direction.upper()}")
            print(f"  Parameters: torque={max_torque}, load_increase={max_load_increase}, return={return_to_center}, set_middle={set_as_middle}")
            
            # Calibrate single limit
            result = self.calibrate_single_limit(
                joint_name=joint_name,
                direction=direction,
                max_torque=max_torque,
                max_load_increase=max_load_increase,
                return_to_center=return_to_center,
                set_as_middle=set_as_middle
            )
            
            if result:
                calibrated_joints.add(joint_name)
                print(f"  ✓ {joint_name} {direction} calibrated: {result}")
                
                # Check if we have both min and max for this joint
                if joint_name in self.calibration_data:
                    joint_data = self.calibration_data[joint_name]
                    if 'detected_min' in joint_data and 'detected_max' in joint_data:
                        # Both limits found - finalize and write to EEPROM
                        detected_min = joint_data['detected_min']
                        detected_max = joint_data['detected_max']
                        joint_data['detected_range'] = abs(detected_max - detected_min)
                        joint_data['timestamp'] = datetime.now().isoformat()
                        
                        print(f"\n  ✓ Both limits found for {joint_name}!")
                        print(f"    Min: {detected_min}, Max: {detected_max}, Range: {joint_data['detected_range']}")
                        print(f"    Writing to servo EEPROM...")
                        
                        write_result = self.driver.set_joint_limits(joint_name, detected_min, detected_max)
                        if write_result['success']:
                            print(f"    ✓ Limits saved to servo {joint_data['servo_id']}")
                            joint_data['limits_written'] = True
                        else:
                            print(f"    ✗ Failed to write limits: {write_result['error']}")
                            joint_data['limits_written'] = False
                            joint_data['write_error'] = write_result['error']
            else:
                print(f"  ✗ {joint_name} {direction} calibration failed")
            
            time.sleep(0.5)
            
        print("\n" + "="*70)
        print("ALL JOINTS CALIBRATION COMPLETE")
        print("="*70)
    
    def save_calibration_log(self, filename=None):
        """
        Save calibration data to a JSON log file
        
        Args:
            filename: Optional filename. If None, generates timestamped filename
        """
        # Create log directory if it doesn't exist
        os.makedirs(LOG_DIR, exist_ok=True)
        
        # Generate filename if not provided
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"calibration_{timestamp}.json"
        
        filepath = os.path.join(LOG_DIR, filename)
        
        # Prepare log data
        log_data = {
            'calibration_timestamp': datetime.now().isoformat(),
            'total_joints': len(self.calibration_data),
            'joints': self.calibration_data
        }
        
        # Save to file
        with open(filepath, 'w') as f:
            json.dump(log_data, f, indent=2)
        
        print(f"\n✓ Calibration log saved to: {filepath}")
        return filepath
    
    def close(self):
        """Close driver connection"""
        self.driver.close()

# ============================================================================
# MAIN
# ============================================================================

def main():
    """Main entry point"""
    import argparse
    
    print("="*70)
    print("ORCA Hand Servo Auto-Calibration Tool")
    print("="*70)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Auto-calibrate ORCA Hand servos')
    parser.add_argument('--joint', type=str, help='Calibrate a specific joint (e.g., right_wrist)')
    parser.add_argument('--all', action='store_true', help='Calibrate all joints')
    args = parser.parse_args()
    
    # Initialize calibrator with GUI monitoring
    print("\nInitializing servo driver with real-time GUI monitoring...")
    try:
        calibrator = ServoCalibrator(enable_gui=True)
        print("✓ Real-time plotting GUI started")
        print("  Watch the GUI window for live torque, position, and metrics!")
    except Exception as e:
        print(f"Failed to initialize: {e}")
        sys.exit(1)
    
    try:
        if args.all:
            # Calibrate all joints
            calibrator.calibrate_all_joints()
        elif args.joint:
            # Calibrate specific joint
            result = calibrator.calibrate_joint(args.joint)
            if not result:
                print(f"\n✗ Calibration failed for {args.joint}")
                sys.exit(1)
        else:
            # Default: show available joints and prompt
            joints = calibrator.driver.list_joints()
            print(f"\nAvailable joints ({len(joints)}):")
            for i, joint in enumerate(joints, 1):
                print(f"  {i:2d}. {joint}")
            
            print("\nOptions:")
            print("  1. Calibrate a specific joint: python calibration.py --joint <joint_name>")
            print("  2. Calibrate all joints: python calibration.py --all")
            
            # Interactive mode
            choice = input("\nCalibrate all joints? (y/n): ").strip().lower()
            if choice == 'y':
                calibrator.calibrate_all_joints()
            else:
                joint_name = input("Enter joint name to calibrate: ").strip()
                if joint_name in joints:
                    calibrator.calibrate_joint(joint_name)
                else:
                    print(f"✗ Invalid joint name: {joint_name}")
                    sys.exit(1)
        
        # Save calibration log
        if calibrator.calibration_data:
            calibrator.save_calibration_log()
            
            print("\n" + "="*70)
            print("CALIBRATION SUMMARY")
            print("="*70)
            for joint_name, data in calibrator.calibration_data.items():
                print(f"\n{joint_name} (ID: {data['servo_id']}):")
                print(f"  Factory: {data['factory_min']} - {data['factory_max']}")
                print(f"  Detected: {data['detected_min']} - {data['detected_max']}")
                print(f"  Range: {data['detected_range']}")
            print("="*70)
    
    except KeyboardInterrupt:
        print("\n\n✗ Calibration interrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        calibrator.close()

if __name__ == "__main__":
    main()
