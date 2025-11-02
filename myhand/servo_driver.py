#!/usr/bin/env python
"""
Servo Driver for ORCA Hand
Provides a clean interface to control servos by joint name with comprehensive safety checks
"""

import sys
import time
import threading
from collections import deque
from scservo_sdk import *
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
PYQT_AVAILABLE = True

# Import joint configuration
from joint_config import JOINT_PARAMS, get_joint_params

# ============================================================================
# SERVO CONFIGURATION
# ============================================================================

# Control table addresses
ADDR_STS_GOAL_POSITION      = 42
ADDR_STS_PRESENT_POSITION   = 56
ADDR_STS_PRESENT_LOAD       = 60
ADDR_STS_TORQUE_ENABLE      = 40
ADDR_SCS_MIN_ANGLE_LIMIT    = 9
ADDR_SCS_MAX_ANGLE_LIMIT    = 11

# Servo settings
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyCH343USB0'
protocol_end = 0  # STS/SMS=0, SCS=1

# Safety settings
MAX_TORQUE_THRESHOLD = 150  # Default maximum safe torque (0-1000)
MAX_LOAD_INCREASE = 1    # Default maximum allowed load increase during movement
MIN_MOVEMENT_THRESHOLD = 1  # Minimum position change to consider servo is moving
TORQUE_CHECK_ENABLED = True  # Enable/disable torque monitoring
SAFETY_CHECK_INTERVAL = 0.01  # High-speed safety check interval (10ms = 100Hz)

# Note: Joint-specific safety settings are now in joint_config.py (JOINT_PARAMS)

# ============================================================================
# JOINT TO SERVO ID MAPPING
# ============================================================================
JOINT_TO_SERVO_MAP = {
    "right_middle_pip": 1,        # 中尖背/腹 - Middle fingertip back/palm
    "right_pinky_mcp": 2,         # 尾指背/腹 - Pinky finger back/palm
    "right_pinky_pip": 3,         # 尾尖背/腹 - Pinky tip back/palm
    "right_pinky_abd": 4,         # 尾左/右 - Pinky left/right abduction
    "right_middle_abd": 5,        # 中左/右 - Middle left/right abduction
    "right_index_abd": 6,         # 食左/右 - Index left/right abduction
    "right_index_mcp": 7,         # 食指背/腹 - Index finger back/palm
    "right_index_pip": 8,         # 食尖背/腹 - Index tip back/palm
    "right_thumb_pip": 9,         # 姆指背/腹 - Thumb finger back/palm (same as 10)
    "right_thumb_tip": 10,        # 姆尖背/腹 - Thumb tip back/palm
    "right_thumb_abd": 11,        # 姆下/上 - Thumb down/up abduction
    "right_thumb_mcp": 12,        # 姆右/左 - Thumb right/left (DIP joint)
    "right_ring_abd": 13,         # 無右/左 - Ring right/left abduction
    "right_ring_pip": 14,         # 無尖背/腹 - Ring tip back/palm
    "right_ring_mcp": 15,         # 無背/腹 - Ring finger back/palm
    "right_middle_mcp": 16,       # 中背/腹 - Middle finger back/palm
    "right_wrist": 17,            # 手後/前 - Hand back/front (wrist rotation)
}

# Reverse mapping for convenience
SERVO_TO_JOINT_MAP = {v: k for k, v in JOINT_TO_SERVO_MAP.items()}


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def get_joint_safety_config(joint_name):
    """
    Get safety configuration for a specific joint
    (Wrapper for backward compatibility - uses joint_config.py)
    
    Args:
        joint_name: Name of the joint
        
    Returns:
        dict: Safety configuration with 'max_torque_threshold' and 'max_load_increase'
    """
    return get_joint_params(joint_name)


# ============================================================================
# SAFETY MONITOR CLASS
# ============================================================================

class SafetyMonitor:
    """
    High-speed safety monitor that runs in a separate thread
    Monitors servo torque and position in real-time with callback support
    """
    
    def __init__(self, driver, servo_id, target_position, max_torque_threshold, max_load_increase=None, callback=None):
        """
        Initialize safety monitor
        
        Args:
            driver: ServoDriver instance
            servo_id: Servo ID to monitor
            target_position: Target position for movement
            max_torque_threshold: Maximum allowed torque
            max_load_increase: Maximum allowed load increase in short time (uses global default if None)
            callback: Optional callback function(data_dict) called on each check
        """
        self.driver = driver
        self.servo_id = servo_id
        self.target_position = target_position
        self.max_torque_threshold = max_torque_threshold
        self.max_load_increase = max_load_increase if max_load_increase is not None else MAX_LOAD_INCREASE
        self.callback = callback
        
        # Monitoring state
        self.running = False
        self.thread = None
        self.safety_triggered = False
        self.error_message = None
        
        # Data collection for plotting
        self.data_log = deque(maxlen=500)  # Keep last 500 samples
        self.start_time = None
        
        # Final results
        self.final_position = None
        self.final_load = None
        self.max_load_observed = 0
        
        # Movement tracking
        self.prev_position = None
        self.baseline_load = None
        self.consecutive_stalls = 0
        
        # Short-term load history for rate of change detection
        self.load_history = deque(maxlen=10)  # Keep last 10 samples (~100ms at 10ms interval)
    
    def start(self):
        """Start the safety monitoring thread"""
        if self.running:
            return
        
        self.running = True
        self.start_time = time.time()
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop the safety monitoring thread"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
    
    def _monitor_loop(self):
        """Main monitoring loop running in separate thread"""
        # Initialize
        time.sleep(0.05)  # Allow movement to start
        self.prev_position = self.driver._read_position(self.servo_id)
        initial_load, _ = self.driver._read_load(self.servo_id)
        self.baseline_load = initial_load
        self.max_load_observed = initial_load
        
        # Initialize load history
        self.load_history.clear()
        self.load_history.append(initial_load)
        
        iteration = 0
        
        while self.running:
            try:
                iteration += 1
                timestamp = time.time() - self.start_time
                
                # Read current state
                current_pos = self.driver._read_position(self.servo_id)
                load_value, direction = self.driver._read_load(self.servo_id)
                
                # Add to load history
                self.load_history.append(load_value)
                
                # Update max load
                self.max_load_observed = max(self.max_load_observed, load_value)
                
                # Calculate metrics
                position_change = abs(current_pos - self.prev_position)
                distance_to_target = abs(current_pos - self.target_position)
                
                # Calculate short-term load increase (compare to oldest value in history)
                if len(self.load_history) >= 2:
                    load_increase = load_value - min(self.load_history)  # Increase from min in recent history
                else:
                    load_increase = 0
                
                # Log data for plotting
                data_point = {
                    'timestamp': timestamp,
                    'iteration': iteration,
                    'position': current_pos,
                    'load': load_value,
                    'load_direction': 'CW' if direction == 1 else 'CCW',
                    'load_increase': load_increase,
                    'baseline_load': self.baseline_load,
                    'position_change': position_change,
                    'distance_to_target': distance_to_target,
                    'torque_threshold': self.max_torque_threshold,
                    'consecutive_stalls': self.consecutive_stalls
                }
                self.data_log.append(data_point)
                
                # Call callback if provided
                if self.callback:
                    try:
                        self.callback(data_point)
                    except Exception as e:
                        print(f"Warning: Callback error: {e}")
                
                # Safety Check 1: Absolute torque threshold
                if load_value > self.max_torque_threshold:
                    self.safety_triggered = True
                    self.error_message = f'High torque detected: {load_value} (threshold: {self.max_torque_threshold})'
                    self.final_position = current_pos
                    self.final_load = load_value
                    self.driver._disable_torque(self.servo_id)
                    self.running = False
                    break
                
                # Safety Check 2: Rapid load increase in short time window
                if len(self.load_history) >= 5 and load_increase > self.max_load_increase:
                    self.safety_triggered = True
                    self.error_message = f'Rapid load increase detected: {load_increase} in {len(self.load_history)*SAFETY_CHECK_INTERVAL*1000:.0f}ms (threshold: {self.max_load_increase})'
                    self.final_position = current_pos
                    self.final_load = load_value
                    self.driver._disable_torque(self.servo_id)
                    self.running = False
                    break
                
                # Track stall for diagnostics
                if position_change < MIN_MOVEMENT_THRESHOLD:
                    self.consecutive_stalls += 1
                else:
                    self.consecutive_stalls = 0
                
                # Check if reached target
                if distance_to_target < 10:
                    # Successfully reached target
                    self.final_position = current_pos
                    self.final_load = load_value
                    self.running = False
                    break
                
                # Update tracking
                self.prev_position = current_pos
                
                # High-speed checking
                time.sleep(SAFETY_CHECK_INTERVAL)
                
            except Exception as e:
                self.safety_triggered = True
                self.error_message = f'Monitor exception: {str(e)}'
                self.running = False
                break
        
        # Final read if not already set
        if self.final_position is None:
            self.final_position = self.driver._read_position(self.servo_id)
            self.final_load, _ = self.driver._read_load(self.servo_id)
    
    def get_result(self):
        """Get monitoring result"""
        return {
            'success': not self.safety_triggered,
            'error': self.error_message,
            'final_position': self.final_position,
            'final_load': self.final_load,
            'max_load_observed': self.max_load_observed,
            'data_log': list(self.data_log)
        }


# ============================================================================
# SERVO DRIVER CLASS
# ============================================================================

class ServoDriver:
    """
    Servo driver with safety checks for ORCA Hand
    
    Usage:
        driver = ServoDriver(enable_gui=True)  # Enable real-time plotting
        result = driver.move_joint("right_wrist", 2048, speed=500)
        if result['success']:
            print(f"Moved to position: {result['final_position']}")
        else:
            print(f"Error: {result['error']}")
        driver.close()
    """
    
    def __init__(self, device_name=DEVICENAME, baudrate=BAUDRATE, enable_gui=False):
        """
        Initialize servo driver
        
        Args:
            device_name: Serial device path
            baudrate: Communication baudrate
            enable_gui: Enable real-time GUI plotting (requires PyQt5)
        """
        self.portHandler = PortHandler(device_name)
        self.packetHandler = PacketHandler(protocol_end)
        self.servo_limits = {}
        self.device_name = device_name
        self.baudrate = baudrate
        
        # GUI monitoring
        self.enable_gui = enable_gui
        self.gui_window = None
        self.gui_app = None
        
        # Initialize sync read/write for position
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler,
            self.packetHandler,
            ADDR_STS_GOAL_POSITION,
            2  # 2 bytes for position
        )
        self.groupSyncRead = GroupSyncRead(
            self.portHandler,
            self.packetHandler,
            ADDR_STS_PRESENT_POSITION,
            4  # 4 bytes for position+speed
        )
        
        self._connect()
        
        # Start GUI monitoring if enabled
        if self.enable_gui:
            self._start_gui_monitor()
    
    def _connect(self):
        """Connect to servo hardware"""
        # Open port
        if not self.portHandler.openPort():
            raise ConnectionError(f"Failed to open port: {self.device_name}")
        print(f"✓ Opened port: {self.device_name}")
        
        # Set baudrate
        if not self.portHandler.setBaudRate(self.baudrate):
            raise ConnectionError("Failed to set baudrate")
        print(f"✓ Set baudrate: {self.baudrate}")
        
        # Initialize all known servo limits
        print("\n=== Reading Servo Limits ===")
        for servo_id in range(1, 18):  # IDs 1-17
            try:
                self._read_servo_limits(servo_id)
                # Add servo to sync read group
                if not self.groupSyncRead.addParam(servo_id):
                    print(f"[ID:{servo_id:03d}] Warning: groupSyncRead addparam failed")
            except Exception as e:
                print(f"[ID:{servo_id:03d}] Warning: Could not read limits - {e}")
    
    def _read_servo_limits(self, servo_id):
        """Read and store servo limits"""
        min_limit, _, _ = self.packetHandler.read2ByteTxRx(
            self.portHandler, servo_id, ADDR_SCS_MIN_ANGLE_LIMIT
        )
        max_limit, _, _ = self.packetHandler.read2ByteTxRx(
            self.portHandler, servo_id, ADDR_SCS_MAX_ANGLE_LIMIT
        )
        
        inverted = max_limit < min_limit
        
        self.servo_limits[servo_id] = {
            'min': min_limit,
            'max': max_limit,
            'range': abs(max_limit - min_limit),
            'inverted': inverted
        }
        
        status = " (INVERTED)" if inverted else ""
        print(f"[ID:{servo_id:03d}] Min: {min_limit:04d} | Max: {max_limit:04d}{status}")
        
        return min_limit, max_limit, inverted
    
    def _write_servo_limits(self, servo_id, min_limit, max_limit):
        """
        Write min/max angle limits to servo EEPROM
        
        Args:
            servo_id: Servo ID
            min_limit: Minimum angle limit (0-4095)
            max_limit: Maximum angle limit (0-4095)
            
        Returns:
            tuple: (success, error_message)
        """
        # Disable torque first (required for EEPROM write)
        if not self._disable_torque(servo_id):
            return False, "Failed to disable torque"
        
        time.sleep(0.1)  # Wait for torque to be disabled
        
        # Write minimum limit
        comm_result, error = self.packetHandler.write2ByteTxRx(
            self.portHandler, servo_id, ADDR_SCS_MIN_ANGLE_LIMIT, min_limit
        )
        if comm_result != COMM_SUCCESS or error != 0:
            return False, f"Failed to write min limit: {self.packetHandler.getTxRxResult(comm_result)}"
        
        time.sleep(0.05)  # Small delay between EEPROM writes
        
        # Write maximum limit
        comm_result, error = self.packetHandler.write2ByteTxRx(
            self.portHandler, servo_id, ADDR_SCS_MAX_ANGLE_LIMIT, max_limit
        )
        if comm_result != COMM_SUCCESS or error != 0:
            return False, f"Failed to write max limit: {self.packetHandler.getTxRxResult(comm_result)}"
        
        time.sleep(0.1)  # Wait for EEPROM write to complete
        
        # Re-enable torque
        if not self._enable_torque(servo_id):
            return False, "Failed to re-enable torque"
        
        # Update cached limits
        inverted = max_limit < min_limit
        self.servo_limits[servo_id] = {
            'min': min_limit,
            'max': max_limit,
            'range': abs(max_limit - min_limit),
            'inverted': inverted
        }
        
        return True, None
    
    def _read_position(self, servo_id):
        """Read current servo position using sync read"""
        # Trigger sync read
        comm_result = self.groupSyncRead.txRxPacket()
        if comm_result != COMM_SUCCESS:
            # Fallback to individual read
            position_speed, _, _ = self.packetHandler.read4ByteTxRx(
                self.portHandler, servo_id, ADDR_STS_PRESENT_POSITION
            )
            position = SCS_LOWORD(position_speed)
            return position
        
        # Get data from sync read
        if self.groupSyncRead.isAvailable(servo_id, ADDR_STS_PRESENT_POSITION, 4):
            position_speed = self.groupSyncRead.getData(servo_id, ADDR_STS_PRESENT_POSITION, 4)
            position = SCS_LOWORD(position_speed)
            return position
        else:
            # Fallback to individual read
            position_speed, _, _ = self.packetHandler.read4ByteTxRx(
                self.portHandler, servo_id, ADDR_STS_PRESENT_POSITION
            )
            position = SCS_LOWORD(position_speed)
            return position
    
    def _read_load(self, servo_id):
        """
        Read current servo load (torque)
        Returns: (load_value, direction) where direction is 1 (CW) or -1 (CCW)
        """
        load_raw, _, _ = self.packetHandler.read2ByteTxRx(
            self.portHandler, servo_id, ADDR_STS_PRESENT_LOAD
        )
        
        # Bit 10 indicates direction: 0 = CCW, 1 = CW
        direction = 1 if (load_raw & 0x0400) else -1
        # Lower 10 bits contain the load value (0-1023)
        load_value = load_raw & 0x03FF
        
        return load_value, direction
    
    def _enable_torque(self, servo_id):
        """Enable servo torque"""
        comm_result, error = self.packetHandler.write1ByteTxRx(
            self.portHandler, servo_id, ADDR_STS_TORQUE_ENABLE, 1
        )
        return comm_result == COMM_SUCCESS and error == 0
    
    def _disable_torque(self, servo_id):
        """Disable servo torque"""
        comm_result, error = self.packetHandler.write1ByteTxRx(
            self.portHandler, servo_id, ADDR_STS_TORQUE_ENABLE, 0
        )
        return comm_result == COMM_SUCCESS and error == 0
    
    def _write_position(self, servo_id, position):
        """Write goal position to servo using sync write"""
        # Prepare parameter (2 bytes: low byte, high byte)
        param_goal_position = [SCS_LOBYTE(position), SCS_HIBYTE(position)]
        
        # Add parameter to sync write
        if not self.groupSyncWrite.addParam(servo_id, bytes(param_goal_position)):
            return False, f"Failed to add parameter for servo {servo_id}"
        
        # Transmit sync write packet
        comm_result = self.groupSyncWrite.txPacket()
        
        # Clear parameter storage
        self.groupSyncWrite.clearParam()
        
        if comm_result != COMM_SUCCESS:
            return False, f"Communication error: {self.packetHandler.getTxRxResult(comm_result)}"
        
        return True, None
    
    def _check_safety(self, servo_id, target_position, monitor_torque=True, max_torque_threshold=None, max_load_increase=None, callback=None):
        """
        Monitor servo movement for safety issues using threaded high-speed monitoring
        
        Args:
            servo_id: Servo ID
            target_position: Target position
            monitor_torque: Enable torque monitoring
            max_torque_threshold: Override MAX_TORQUE_THRESHOLD for this check
            max_load_increase: Override MAX_LOAD_INCREASE for this check
            callback: Optional callback function(data_dict) called on each check
        
        Returns:
            dict: {
                'success': bool,
                'error': str or None,
                'final_position': int,
                'final_load': int,
                'max_load_observed': int,
                'data_log': list of monitoring data (optional)
            }
        """
        # Use provided threshold or default
        torque_threshold = max_torque_threshold if max_torque_threshold is not None else MAX_TORQUE_THRESHOLD
        load_increase_threshold = max_load_increase if max_load_increase is not None else MAX_LOAD_INCREASE
        
        if not monitor_torque or not TORQUE_CHECK_ENABLED:
            time.sleep(0.5)  # Simple wait
            final_pos = self._read_position(servo_id)
            final_load, _ = self._read_load(servo_id)
            return {
                'success': True,
                'error': None,
                'final_position': final_pos,
                'final_load': final_load,
                'max_load_observed': final_load,
                'data_log': []
            }
        
        # Create and start safety monitor thread
        monitor = SafetyMonitor(
            driver=self,
            servo_id=servo_id,
            target_position=target_position,
            max_torque_threshold=torque_threshold,
            max_load_increase=load_increase_threshold,
            callback=callback
        )
        
        monitor.start()
        
        # Wait for monitoring to complete (max 3 seconds)
        timeout = 3.0
        start_time = time.time()
        
        while monitor.running and (time.time() - start_time) < timeout:
            time.sleep(0.05)  # Check every 50ms
        
        # Stop monitor if still running (timeout)
        if monitor.running:
            monitor.stop()
        
        # Get result
        result = monitor.get_result()
        
        return result
    
    def get_joint_status(self, joint_name):
        """
        Get current status of a joint
        
        Args:
            joint_name: Name of the joint (e.g., "right_wrist")
        
        Returns:
            dict: {
                'success': bool,
                'error': str or None,
                'joint_name': str,
                'servo_id': int,
                'position': int,
                'load': int,
                'load_direction': str,
                'limits': dict
            }
        """
        # Validate joint name
        if joint_name not in JOINT_TO_SERVO_MAP:
            return {
                'success': False,
                'error': f'Unknown joint name: {joint_name}. Valid joints: {list(JOINT_TO_SERVO_MAP.keys())}',
                'joint_name': joint_name,
                'servo_id': None,
                'position': None,
                'load': None,
                'load_direction': None,
                'limits': None
            }
        
        servo_id = JOINT_TO_SERVO_MAP[joint_name]
        
        try:
            # Read position
            position = self._read_position(servo_id)
            
            # Read load
            load_value, direction = self._read_load(servo_id)
            load_dir_str = 'CW' if direction == 1 else 'CCW'
            
            # Get limits
            limits = self.servo_limits.get(servo_id, {})
            
            return {
                'success': True,
                'error': None,
                'joint_name': joint_name,
                'servo_id': servo_id,
                'position': position,
                'load': load_value,
                'load_direction': load_dir_str,
                'limits': limits
            }
        
        except Exception as e:
            return {
                'success': False,
                'error': f'Failed to read servo: {str(e)}',
                'joint_name': joint_name,
                'servo_id': servo_id,
                'position': None,
                'load': None,
                'load_direction': None,
                'limits': None
            }
    
    def move_joint(self, joint_name, target_position, speed=500, monitor_torque=True, max_torque_threshold=None, max_load_increase=None, safety_callback=None):
        """
        Move a joint to a target position with safety checks
        
        Args:
            joint_name: Name of the joint (e.g., "right_wrist")
            target_position: Target position in servo units (0-4095)
            speed: Movement speed (not used, kept for compatibility)
            monitor_torque: Enable torque monitoring during movement
            max_torque_threshold: Override joint-specific MAX_TORQUE_THRESHOLD for this movement
            max_load_increase: Override joint-specific MAX_LOAD_INCREASE for this movement
            safety_callback: Optional callback function(data_dict) for real-time monitoring data
        
        Returns:
            dict: {
                'success': bool,
                'error': str or None,
                'joint_name': str,
                'servo_id': int,
                'target_position': int,
                'initial_position': int,
                'final_position': int,
                'initial_load': int,
                'final_load': int,
                'max_load_observed': int,
                'limits': dict,
                'data_log': list (if monitored)
            }
        """
        # Validate joint name
        if joint_name not in JOINT_TO_SERVO_MAP:
            return {
                'success': False,
                'error': f'Unknown joint name: {joint_name}. Valid joints: {list(JOINT_TO_SERVO_MAP.keys())}',
                'joint_name': joint_name,
                'servo_id': None,
                'target_position': target_position,
                'initial_position': None,
                'final_position': None,
                'initial_load': None,
                'final_load': None,
                'max_load_observed': None,
                'limits': None
            }
        
        servo_id = JOINT_TO_SERVO_MAP[joint_name]
        
        # Get servo limits
        limits = self.servo_limits.get(servo_id)
        if not limits:
            return {
                'success': False,
                'error': f'Servo limits not available for ID {servo_id}',
                'joint_name': joint_name,
                'servo_id': servo_id,
                'target_position': target_position,
                'initial_position': None,
                'final_position': None,
                'initial_load': None,
                'final_load': None,
                'max_load_observed': None,
                'limits': None
            }
        
        # Validate target position is within limits
        min_pos = min(limits['min'], limits['max'])
        max_pos = max(limits['min'], limits['max'])
        if not (min_pos <= target_position <= max_pos):
            return {
                'success': False,
                'error': f'Target position {target_position} out of range [{min_pos}, {max_pos}]',
                'joint_name': joint_name,
                'servo_id': servo_id,
                'target_position': target_position,
                'initial_position': None,
                'final_position': None,
                'initial_load': None,
                'final_load': None,
                'max_load_observed': None,
                'limits': limits
            }
        
        try:
            # Read initial state
            initial_position = self._read_position(servo_id)
            initial_load, _ = self._read_load(servo_id)
            
            # Enable torque
            if not self._enable_torque(servo_id):
                return {
                    'success': False,
                    'error': 'Failed to enable torque',
                    'joint_name': joint_name,
                    'servo_id': servo_id,
                    'target_position': target_position,
                    'initial_position': initial_position,
                    'final_position': initial_position,
                    'initial_load': initial_load,
                    'final_load': initial_load,
                    'max_load_observed': initial_load,
                    'limits': limits
                }
            
            # Write target position
            success, error = self._write_position(servo_id, target_position)
            if not success:
                return {
                    'success': False,
                    'error': error,
                    'joint_name': joint_name,
                    'servo_id': servo_id,
                    'target_position': target_position,
                    'initial_position': initial_position,
                    'final_position': initial_position,
                    'initial_load': initial_load,
                    'final_load': initial_load,
                    'max_load_observed': initial_load,
                    'limits': limits
                }
            
            # Get joint-specific safety configuration
            safety_config = get_joint_safety_config(joint_name)
            
            # Use provided thresholds or joint-specific defaults
            torque_threshold = max_torque_threshold if max_torque_threshold is not None else safety_config['max_torque_threshold']
            load_increase_threshold = max_load_increase if max_load_increase is not None else safety_config['max_load_increase']
            
            # Register joint with GUI if enabled
            if self.enable_gui and self.gui_window:
                self.gui_window.register_joint(joint_name)
            
            # Create GUI callback wrapper if GUI is enabled or user callback provided
            combined_callback = None
            if self.enable_gui and self.gui_window:
                def gui_callback_wrapper(data_point):
                    # Update GUI with real-time data
                    self.gui_window.update_data(joint_name, data_point)
                    # Call user callback if provided
                    if safety_callback:
                        safety_callback(data_point)
                combined_callback = gui_callback_wrapper
            elif safety_callback:
                combined_callback = safety_callback
            
            # Monitor movement with safety checks
            safety_result = self._check_safety(servo_id, target_position, monitor_torque, torque_threshold, load_increase_threshold, combined_callback)
            
            result = {
                'success': safety_result['success'],
                'error': safety_result['error'],
                'joint_name': joint_name,
                'servo_id': servo_id,
                'target_position': target_position,
                'initial_position': initial_position,
                'final_position': safety_result['final_position'],
                'initial_load': initial_load,
                'final_load': safety_result['final_load'],
                'max_load_observed': safety_result['max_load_observed'],
                'limits': limits,
                'data_log': safety_result.get('data_log', [])
            }
            
            return result
        
        except Exception as e:
            return {
                'success': False,
                'error': f'Exception during movement: {str(e)}',
                'joint_name': joint_name,
                'servo_id': servo_id,
                'target_position': target_position,
                'initial_position': None,
                'final_position': None,
                'initial_load': None,
                'final_load': None,
                'max_load_observed': None,
                'limits': limits
            }
    
    def move_joints_batch(self, joint_positions, monitor_torque=True):
        """
        Move multiple joints simultaneously using sync write
        
        Args:
            joint_positions: dict of {joint_name: target_position}
            monitor_torque: Enable torque monitoring during movement
        
        Returns:
            dict: {
                'success': bool,
                'error': str or None,
                'results': dict of {joint_name: result_dict}
            }
        """
        results = {}
        
        # Validate all joint names first
        for joint_name in joint_positions.keys():
            if joint_name not in JOINT_TO_SERVO_MAP:
                return {
                    'success': False,
                    'error': f'Unknown joint name: {joint_name}',
                    'results': {}
                }
        
        # Enable torque for all servos
        for joint_name in joint_positions.keys():
            servo_id = JOINT_TO_SERVO_MAP[joint_name]
            if not self._enable_torque(servo_id):
                return {
                    'success': False,
                    'error': f'Failed to enable torque for {joint_name}',
                    'results': results
                }
        
        # Add all positions to sync write
        for joint_name, target_pos in joint_positions.items():
            servo_id = JOINT_TO_SERVO_MAP[joint_name]
            param_goal_position = [SCS_LOBYTE(target_pos), SCS_HIBYTE(target_pos)]
            
            if not self.groupSyncWrite.addParam(servo_id, bytes(param_goal_position)):
                self.groupSyncWrite.clearParam()
                return {
                    'success': False,
                    'error': f'Failed to add parameter for {joint_name}',
                    'results': results
                }
        
        # Send sync write
        comm_result = self.groupSyncWrite.txPacket()
        self.groupSyncWrite.clearParam()
        
        if comm_result != COMM_SUCCESS:
            return {
                'success': False,
                'error': f'Sync write failed: {self.packetHandler.getTxRxResult(comm_result)}',
                'results': results
            }
        
        # Monitor each joint's safety
        time.sleep(0.5)  # Allow movement to complete
        
        all_success = True
        for joint_name, target_pos in joint_positions.items():
            servo_id = JOINT_TO_SERVO_MAP[joint_name]
            
            # Check final position and load
            final_pos = self._read_position(servo_id)
            final_load, _ = self._read_load(servo_id)
            
            result = {
                'success': True,
                'final_position': final_pos,
                'final_load': final_load,
                'error': None
            }
            
            # Check if position was reached
            if abs(final_pos - target_pos) > 50:
                result['success'] = False
                result['error'] = f'Position not reached (target: {target_pos}, actual: {final_pos})'
                all_success = False
            
            results[joint_name] = result
        
        return {
            'success': all_success,
            'error': None if all_success else 'Some joints failed to reach target',
            'results': results
        }
    
    def list_joints(self):
        """
        List all available joints
        
        Returns:
            list: List of joint names
        """
        return list(JOINT_TO_SERVO_MAP.keys())
    
    def set_joint_limits(self, joint_name, min_limit, max_limit):
        """
        Set the min/max angle limits for a joint in servo EEPROM
        
        Args:
            joint_name: Name of the joint (e.g., "right_wrist")
            min_limit: Minimum angle limit (0-4095)
            max_limit: Maximum angle limit (0-4095)
        
        Returns:
            dict: {
                'success': bool,
                'error': str or None,
                'joint_name': str,
                'servo_id': int,
                'min_limit': int,
                'max_limit': int
            }
        """
        # Validate joint name
        if joint_name not in JOINT_TO_SERVO_MAP:
            return {
                'success': False,
                'error': f'Unknown joint name: {joint_name}',
                'joint_name': joint_name,
                'servo_id': None,
                'min_limit': None,
                'max_limit': None
            }
        
        servo_id = JOINT_TO_SERVO_MAP[joint_name]
        
        # Validate limits are in valid range
        if not (0 <= min_limit <= 4095) or not (0 <= max_limit <= 4095):
            return {
                'success': False,
                'error': f'Limits must be in range [0, 4095]',
                'joint_name': joint_name,
                'servo_id': servo_id,
                'min_limit': min_limit,
                'max_limit': max_limit
            }
        
        # Write limits to servo
        success, error = self._write_servo_limits(servo_id, min_limit, max_limit)
        
        return {
            'success': success,
            'error': error,
            'joint_name': joint_name,
            'servo_id': servo_id,
            'min_limit': min_limit,
            'max_limit': max_limit
        }
    
    def _start_gui_monitor(self):
        def run_gui():
            """Run GUI in separate thread"""
            try:
                # Import GUI module
                from realtime_monitor_gui import MonitorGUI
                
                # Create QApplication if not exists
                app = QApplication.instance()
                if app is None:
                    app = QApplication(sys.argv)
                
                self.gui_app = app
                
                # Create GUI window
                self.gui_window = MonitorGUI()
                self.gui_window.show()
                
                print("✓ Real-time plotting GUI started")
                
                # Start event loop
                app.exec_()
                
            except ImportError as e:
                print(f"⚠️  Failed to import GUI module: {e}")
                print("    Make sure realtime_monitor_gui.py is in the same directory")
            except Exception as e:
                print(f"⚠️  GUI error: {e}")
        
        # Start GUI thread
        gui_thread = threading.Thread(target=run_gui, daemon=True)
        gui_thread.start()
        time.sleep(1.0)  # Give GUI time to initialize
    
    def close(self):
        """Close servo connection"""
        # Close GUI if running
        if self.gui_window:
            try:
                if self.gui_app:
                    self.gui_app.quit()
            except:
                pass
        
        # Clear sync read/write parameter storage
        self.groupSyncRead.clearParam()
        self.groupSyncWrite.clearParam()
        self.portHandler.closePort()
        print("\n✓ Connection closed")


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

def main():
    """Example usage of ServoDriver with real-time plotting GUI"""
    print("="*70)
    print("ORCA Hand Servo Driver - Real-Time Plotting Example")
    print("="*70)
    
    # Initialize driver with GUI monitoring enabled
    print("\n[1/4] Initializing servo driver with real-time plotting GUI...")
    try:
        driver = ServoDriver(enable_gui=True)
    except Exception as e:
        print(f"Failed to initialize: {e}")
        sys.exit(1)
    
    print("\n✓ Real-time plotting window opened")
    print("  Watch the GUI for live torque, position, and metrics data!")
    
    # List available joints
    print("\n[2/4] Available joints:")
    joints = driver.list_joints()
    for i, joint in enumerate(joints, 1):
        print(f"  {i:2d}. {joint}")
    
    # Get status
    print("\n[3/4] Reading joint status...")
    status = driver.get_joint_status("right_wrist")
    
    if status['success']:
        print(f"  ✓ {status['joint_name']} (ID: {status['servo_id']})")
        print(f"    Position: {status['position']}")
        print(f"    Load: {status['load']} ({status['load_direction']})")
        print(f"    Limits: {status['limits']['min']} - {status['limits']['max']}")
    else:
        print(f"  ✗ Error: {status['error']}")
        driver.close()
        sys.exit(1)
    
    # Move joint with real-time plotting
    print("\n[4/4] Moving joint with real-time plotting...")
    print("  → Watch the GUI for live data visualization!")
    
    limits = status['limits']
    min_pos = limits['min']
    max_pos = limits['max']
    center = (min_pos + max_pos) // 2
    
    # Move to MIN
    print(f"\n  Step 1: Moving to MIN position ({min_pos})...")
    result = driver.move_joint("right_wrist", min_pos)
    if result['success']:
        print(f"    ✓ Reached: {result['final_position']} (load: {result['final_load']})")
        print(f"    Max load observed: {result['max_load_observed']}")
        print(f"    Data points collected: {len(result['data_log'])}")
    else:
        print(f"    ✗ Failed: {result['error']}")
    time.sleep(2.0)
    
    # Move to MAX
    print(f"\n  Step 2: Moving to MAX position ({max_pos})...")
    result = driver.move_joint("right_wrist", max_pos)
    if result['success']:
        print(f"    ✓ Reached: {result['final_position']} (load: {result['final_load']})")
        print(f"    Max load observed: {result['max_load_observed']}")
        print(f"    Data points collected: {len(result['data_log'])}")
    else:
        print(f"    ✗ Failed: {result['error']}")
    time.sleep(2.0)
    
    # Move to CENTER
    print(f"\n  Step 3: Moving to CENTER position ({center})...")
    result = driver.move_joint("right_wrist", center)
    if result['success']:
        print(f"    ✓ Reached: {result['final_position']} (load: {result['final_load']})")
        print(f"    Position error: {abs(result['final_position'] - result['target_position'])} units")
        print(f"    Max load observed: {result['max_load_observed']}")
        print(f"    Data points collected: {len(result['data_log'])}")
    else:
        print(f"    ✗ Failed: {result['error']}")
    
    # Summary
    print("\n" + "="*70)
    print("Example Complete!")
    print("  ✓ Real-time plotting GUI displayed torque, position, and metrics")
    print("  ✓ Safety monitoring active during all movements")
    print("  ✓ Data collection logged for analysis")
    print("\n  Keep the GUI window open to view the plots...")
    print("  Press Ctrl+C to close")
    print("="*70)
    
    # Keep running to view GUI
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n✓ Shutting down...")
        driver.close()


if __name__ == "__main__":
    main()
