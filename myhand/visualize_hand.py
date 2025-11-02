#!/usr/bin/env python
"""
MuJoCo Visualization for ORCA Hand
Reads joint positions from servo hardware and displays in MuJoCo simulator
"""

import os
import sys
import numpy as np
import mujoco
import mujoco.viewer
from scservo_sdk import *

# ============================================================================
# SERVO CONFIGURATION
# ============================================================================

# Control table addresses
ADDR_STS_PRESENT_POSITION  = 56
ADDR_SCS_MIN_ANGLE_LIMIT   = 9
ADDR_SCS_MAX_ANGLE_LIMIT   = 11

# Servo settings
SERVO_IDS = list(range(1, 18))  # Servo IDs: 1-17
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyACM0'
protocol_end = 0  # STS/SMS=0, SCS=1

# ============================================================================
# SERVO ID TO JOINT MAPPING
# ============================================================================
# Maps servo ID to MuJoCo joint name
# Based on actual hardware configuration from servo limit data
SERVO_TO_JOINT_MAP = {
    1:  "right_middle_pip",       # 中尖背/腹 - Middle fingertip back/palm
    2:  "right_pinky_mcp",        # 尾指背/腹 - Pinky finger back/palm
    3:  "right_pinky_pip",        # 尾尖背/腹 - Pinky tip back/palm
    4:  "right_pinky_abd",        # 尾左/右 - Pinky left/right abduction
    5:  "right_middle_abd",       # 中左/右 - Middle left/right abduction
    6:  "right_index_abd",        # 食左/右 - Index left/right abduction
    7:  "right_index_mcp",        # 食指背/腹 - Index finger back/palm
    8:  "right_index_pip",        # 食尖背/腹 - Index tip back/palm
    9:  "right_thumb_pip",        # 姆指背/腹 - Thumb finger back/palm
    10: "right_thumb_dip",        # 姆尖背/腹 - Thumb tip back/palm
    11: "right_thumb_abd",        # 姆下/上 - Thumb down/up abduction
    12: "right_thumb_mcp",        # 姆右/左 - Thumb right/left (DIP joint)
    13: "right_ring_abd",         # 無右/左 - Ring right/left abduction
    14: "right_ring_pip",         # 無尖背/腹 - Ring tip back/palm
    15: "right_ring_mcp",         # 無背/腹 - Ring finger back/palm
    16: "right_middle_mcp",       # 中背/腹 - Middle finger back/palm
    17: "right_wrist",            # 手後/前 - Hand back/front (wrist rotation)
}

# ============================================================================
# MUJOCO JOINT RANGES (radians)
# ============================================================================
# Set per-joint min/max in MuJoCo (radians). If a joint is not present here or
# its value is None, the code falls back to the default 270° (~4.71 rad) centered
# at 0 mapping used previously.
#
# Example:
# MJ_JOINT_LIMITS = {
#     'right_wrist': (-1.57, 1.57),            # wrist: -90° .. +90°
#     'right_index_mcp': (0.0, 1.57),          # index MCP: 0° .. 90°
# }
#
# Populate the entries below to match your MuJoCo joint coordinate frames.
MJ_JOINT_LIMITS = {
    # Wrist: use user's custom setting (kept as-is)
    'right_wrist': (-0.520, 0.34),

    # Thumb (from orcahand_right.mjcf)
    'right_thumb_mcp': (-0.87266, 0.87266),
    'right_thumb_abd': (-1.08211, 0.0),
    'right_thumb_pip': (-0.7944, 1.23),
    'right_thumb_dip': (-0.85384, 1.45),

    # Index
    'right_index_abd': (-1.04577, 0.24577),
    'right_index_mcp': (-0.34907, 1.65806),
    'right_index_pip': (-0.34907, 1.88496),

    # Middle
    'right_middle_abd': (-0.64577, 0.64577),
    'right_middle_mcp': (-0.34907, 1.58825),
    'right_middle_pip': (-0.34907, 1.8675),

    # Ring
    'right_ring_abd': (-0.47577, 0.80577),
    'right_ring_mcp': (-0.34907, 1.58825),
    'right_ring_pip': (-0.34907, 1.8675),

    # Pinky
    'right_pinky_abd': (-0.12244, 1.1691),
    'right_pinky_mcp': (-0.34907, 1.71042),
    'right_pinky_pip': (-0.34907, 1.88496),
}

# ============================================================================
# MUJOCO JOINT INVERT LIST
# ============================================================================
# Some joints in the MuJoCo model have their min/max oriented opposite to the
# desired mapping from servo normalized [0..1]. List joints here to have their
# MuJoCo min/max swapped when converting normalized servo values to radians.
# Populate with joint names from the MuJoCo model. The user indicated these
# were inverted: wrist, index_abd, middle_abd, ring_abd, pinky_abd.
MJ_INVERT_JOINTS = [
    'right_wrist', #
    'right_index_abd', #
    'right_middle_abd', #
    'right_pinky_abd', #
    # 'right_middle_pip' # wrong routing?

]

print(f"MJ_INVERT_JOINTS: {MJ_INVERT_JOINTS}")

# Pre-process MJ_JOINT_LIMITS: for any joint listed in MJ_INVERT_JOINTS,
# swap its (min, max) tuple so downstream code can use MJ_JOINT_LIMITS directly.
_swapped_joints = []
for jname in MJ_INVERT_JOINTS:
    if jname in MJ_JOINT_LIMITS:
        limits = MJ_JOINT_LIMITS[jname]
        if isinstance(limits, (list, tuple)) and len(limits) == 2:
            MJ_JOINT_LIMITS[jname] = (limits[1], limits[0])
            _swapped_joints.append(jname)

if _swapped_joints:
    print(f"MJ_JOINT_LIMITS swapped for: {_swapped_joints}")
else:
    print("No MJ_JOINT_LIMITS entries were swapped (check names in MJ_INVERT_JOINTS)")

# ============================================================================
# SERVO HARDWARE INTERFACE
# ============================================================================

class ServoReader:
    """Reads position data from servo hardware"""
    
    def __init__(self):
        """Initialize servo communication"""
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(protocol_end)
        self.groupSyncRead = GroupSyncRead(
            self.portHandler, 
            self.packetHandler, 
            ADDR_STS_PRESENT_POSITION, 
            4
        )
        
        # Store servo limits
        self.servo_limits = {}
        
        # Connect to servos
        self._connect()
        
    def _connect(self):
        """Connect to servo hardware"""
        # Open port
        if not self.portHandler.openPort():
            print("Failed to open the port")
            print(f"Check if {DEVICENAME} is available")
            sys.exit(1)
        print(f"Succeeded to open port: {DEVICENAME}")
        
        # Set baudrate
        if not self.portHandler.setBaudRate(BAUDRATE):
            print("Failed to change the baudrate")
            sys.exit(1)
        print(f"Succeeded to set baudrate: {BAUDRATE}")
        
        # Read servo limits
        print("\n=== Reading Servo Limits ===")
        for servo_id in SERVO_IDS:
            min_limit, _, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, servo_id, ADDR_SCS_MIN_ANGLE_LIMIT
            )
            max_limit, _, _ = self.packetHandler.read2ByteTxRx(
                self.portHandler, servo_id, ADDR_SCS_MAX_ANGLE_LIMIT
            )
            # Handle possible inverted limits (some servos report max < min)
            inverted = False
            raw_min = min_limit
            raw_max = max_limit

            if max_limit < min_limit:
                # Mark inverted and swap for normalization purposes
                inverted = True
                norm_min = max_limit
                norm_max = min_limit
            else:
                norm_min = min_limit
                norm_max = max_limit

            self.servo_limits[servo_id] = {
                'raw_min': raw_min,
                'raw_max': raw_max,
                'min': norm_min,
                'max': norm_max,
                'range': norm_max - norm_min,
                'inverted': inverted,
            }

            if inverted:
                print(f"[ID:{servo_id:03d}] Min: {min_limit:04d} | Max: {max_limit:04d}  <-- INVERTED")
            else:
                print(f"[ID:{servo_id:03d}] Min: {min_limit:04d} | Max: {max_limit:04d}")
        
        # Add servos to sync read group
        for servo_id in SERVO_IDS:
            if not self.groupSyncRead.addParam(servo_id):
                print(f"[ID:{servo_id:03d}] groupSyncRead addparam failed")
                sys.exit(1)
        
        print("=== Servo initialization complete ===\n")
    
    def read_positions(self):
        """
        Read current positions from all servos
        
        Returns:
            dict: {servo_id: position} in servo units (0-4095)
        """
        positions = {}
        
        # Sync read from all servos
        comm_result = self.groupSyncRead.txRxPacket()
        if comm_result != COMM_SUCCESS:
            print(f"Read error: {self.packetHandler.getTxRxResult(comm_result)}")
            return positions
        
        # Extract position data for each servo
        for servo_id in SERVO_IDS:
            if self.groupSyncRead.isAvailable(servo_id, ADDR_STS_PRESENT_POSITION, 4):
                # Read 4-byte value (position + speed)
                position_speed = self.groupSyncRead.getData(servo_id, ADDR_STS_PRESENT_POSITION, 4)
                # Extract position (lower 16 bits)
                position = SCS_LOWORD(position_speed)
                positions[servo_id] = position
            else:
                print(f"[ID:{servo_id:03d}] Data not available")
        
        return positions
    
    def servo_to_radian(self, servo_id, servo_position):
        """
        Convert servo position to radians based on servo limits
        
        Args:
            servo_id: Servo ID
            servo_position: Servo position value (0-4095)
            
        Returns:
            float: Position in radians, normalized to servo's range
        """
        if servo_id not in self.servo_limits:
            return 0.0

        # Servo limits information (may have been swapped if inverted)
        limits = self.servo_limits[servo_id]
        min_pos = limits['min']
        max_pos = limits['max']
        inverted = limits.get('inverted', False)

        # Map servo -> MuJoCo joint name (if available) so we can use per-joint
        # MuJoCo limits when converting normalized servo values to radians.
        joint_name = SERVO_TO_JOINT_MAP.get(servo_id, None)

        # Normalize to 0-1 range based on stored (possibly swapped) servo limits
        if max_pos == min_pos:
            normalized = 0.5
        else:
            normalized = (servo_position - min_pos) / (max_pos - min_pos)

        # If servo reported inverted limits (raw_max < raw_min), flip the normalized value
        if inverted:
            normalized = 1.0 - normalized

        # Clamp to [0, 1] to avoid overflow when servo moves outside reported limits
        normalized = float(np.clip(normalized, 0.0, 1.0))

        # If the MuJoCo joint has user-specified min/max (radians), use that range.
        joint_limits = None
        if joint_name is not None:
            joint_limits = MJ_JOINT_LIMITS.get(joint_name)

        if joint_limits and isinstance(joint_limits, (list, tuple)) and len(joint_limits) == 2:
            jmin, jmax = float(joint_limits[0]), float(joint_limits[1])
            angle_rad = jmin + normalized * (jmax - jmin)
        else:
            # Fallback: assume ~270° mechanical travel centered at 0
            angle_rad = (normalized - 0.5) * 4.71

        return angle_rad
    
    def close(self):
        """Close servo connection"""
        self.groupSyncRead.clearParam()
        self.portHandler.closePort()
        print("Servo connection closed")

# ============================================================================
# MUJOCO VISUALIZATION
# ============================================================================

class MuJoCoVisualizer:
    """Visualizes hand in MuJoCo based on servo positions"""
    
    def __init__(self, model_path):
        """
        Initialize MuJoCo visualizer
        
        Args:
            model_path: Path to MuJoCo XML scene file
        """
        # Load MuJoCo model
        if not os.path.exists(model_path):
            print(f"Model file not found: {model_path}")
            sys.exit(1)
        
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Create joint name to index mapping
        self.joint_map = {}
        for i in range(self.model.njnt):
            joint_name = self.model.joint(i).name
            self.joint_map[joint_name] = i
        
        print("\n=== MuJoCo Joints Found ===")
        for name, idx in self.joint_map.items():
            print(f"Joint {idx}: {name}")
        print()
        
    def set_joint_positions(self, joint_angles):
        """
        Set joint positions in MuJoCo model
        
        Args:
            joint_angles: dict of {joint_name: angle_in_radians}
        """
        for joint_name, angle in joint_angles.items():
            if joint_name in self.joint_map:
                joint_idx = self.joint_map[joint_name]
                qpos_idx = self.model.jnt_qposadr[joint_idx]
                self.data.qpos[qpos_idx] = angle
    
    def run(self, servo_reader):
        """
        Run visualization loop
        
        Args:
            servo_reader: ServoReader instance
        """
        print("=== Starting MuJoCo Visualization ===")
        print("Press ESC to exit\n")
        
        # Launch viewer
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # Visualization loop
            while viewer.is_running():
                # Read servo positions
                servo_positions = servo_reader.read_positions()
                
                # Convert servo positions to joint angles
                joint_angles = {}
                for servo_id, servo_pos in servo_positions.items():
                    if servo_id in SERVO_TO_JOINT_MAP:
                        joint_name = SERVO_TO_JOINT_MAP[servo_id]
                        angle_rad = servo_reader.servo_to_radian(servo_id, servo_pos)
                        joint_angles[joint_name] = angle_rad
                
                # Update MuJoCo model
                self.set_joint_positions(joint_angles)
                
                # Step simulation
                mujoco.mj_step(self.model, self.data)
                
                # Update viewer
                viewer.sync()
                
                # Print status (every 100 steps)
                if self.data.time % 1.0 < 0.002:  # Approximately every second
                    print(f"Time: {self.data.time:.1f}s | Servos: {len(servo_positions)} | Joints: {len(joint_angles)}")

# ============================================================================
# MAIN
# ============================================================================

def main():
    """Main entry point"""
    # Path to MuJoCo scene file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(
        script_dir, 
        "../orcahand_description/scene_right.xml"
    )
    
    print("="*60)
    print("MuJoCo ORCA Hand Visualizer")
    print("="*60)
    
    # Initialize servo reader
    print("\n[1/2] Initializing servo hardware...")
    try:
        servo_reader = ServoReader()
    except Exception as e:
        print(f"Failed to initialize servos: {e}")
        sys.exit(1)
    
    # Initialize MuJoCo visualizer
    print("\n[2/2] Initializing MuJoCo visualizer...")
    try:
        visualizer = MuJoCoVisualizer(model_path)
    except Exception as e:
        print(f"Failed to initialize MuJoCo: {e}")
        servo_reader.close()
        sys.exit(1)
    
    # Run visualization
    try:
        visualizer.run(servo_reader)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError during visualization: {e}")
    finally:
        servo_reader.close()
        print("\nVisualization stopped")

if __name__ == "__main__":
    main()
