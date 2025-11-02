#!/usr/bin/env python
"""
Joint Configuration for ORCA Hand
Centralized configuration for joint-specific safety and calibration parameters
"""

# ============================================================================
# JOINT-SPECIFIC SAFETY AND CALIBRATION PARAMETERS
# ============================================================================
# Define safety and calibration parameters for each joint or joint group
# This allows easy tuning of thresholds per joint for both runtime and calibration

JOINT_PARAMS = {
    # Wrist needs high torque threshold due to mechanical resistance
    "right_wrist": {
        'max_torque_threshold': 700,
        'max_load_increase': 300,
    },
    
    # ABD joints (abduction/adduction) - moderate threshold
    "right_index_abd": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    },
    "right_middle_abd": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    },
    "right_ring_abd": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    },
    "right_pinky_abd": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    },
    
    # Thumb joints - slightly higher threshold
    "right_thumb_abd": {
        'max_torque_threshold': 180,
        'max_load_increase': 10,
    },
    "right_thumb_mcp": {
        'max_torque_threshold': 180,
        'max_load_increase': 10,
    },
    "right_thumb_pip": {
        'max_torque_threshold': 180,
        'max_load_increase': 10,
    },
    "right_thumb_tip": {
        'max_torque_threshold': 180,
        'max_load_increase': 10,
    },
    
    # MCP joints (metacarpophalangeal) - default threshold
    "right_index_mcp": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    },
    "right_middle_mcp": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    },
    "right_ring_mcp": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    },
    "right_pinky_mcp": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    },
    
    # PIP joints (proximal interphalangeal) - default threshold
    "right_index_pip": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    },
    "right_middle_pip": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    },
    "right_ring_pip": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    },
    "right_pinky_pip": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    },
    
    # Default for any unlisted joints
    "_default": {
        'max_torque_threshold': 150,
        'max_load_increase': 10,
    }
}


def get_joint_params(joint_name):
    """
    Get safety and calibration parameters for a specific joint
    
    Args:
        joint_name: Name of the joint (e.g., "right_wrist")
        
    Returns:
        dict: Parameters with 'max_torque_threshold' and 'max_load_increase'
    """
    if joint_name in JOINT_PARAMS:
        return JOINT_PARAMS[joint_name].copy()
    else:
        # Return default values
        return JOINT_PARAMS['_default'].copy()


# ============================================================================
# CALIBRATION ORDER CONFIGURATION
# ============================================================================
# Define the exact order of calibration operations
# Format: (joint_name, limit_direction, max_torque_threshold, max_load_increase, return_to_center, set_as_middle)
# 
# Parameters:
#   joint_name: Name of the joint to calibrate
#   limit_direction: 'min', 'max', or 'both'
#   max_torque_threshold: Maximum torque threshold for this operation (None = use from JOINT_PARAMS)
#   max_load_increase: Maximum load increase threshold (None = use from JOINT_PARAMS)
#   return_to_center: True/False (move to mean of detected range after finding limit)
#   set_as_middle: True/False (remap servo so detected position becomes 2047 middle)

CALIBRATION_ORDER = [
    # Phase 1: Wrist calibration (high torque needed)
    # (joint_name, direction, max_torque, max_load_increase, return_to_center, set_as_middle)
    ("right_wrist", "min", 200, 20, False, False),
    ("right_wrist", "max", 200, 20, True, True),
    
    # Phase 2: ABD joints MIN (pinky → ring → middle → index)
    # No return to center between fingers to save time
    # Set detected min as middle (2047) for each joint
    ("right_pinky_abd", "min", 150, 10, True, True),
    ("right_ring_abd", "max", 100, 5, True, True),
    ("right_middle_abd", "min", 150, 10, True, True),
    ("right_index_abd", "min", 150, 10, True, True),
    
    # Phase 3: ABD joints MAX (index → middle → ring → pinky)
    # Return to center after each to prepare for next joint
    ("right_index_abd", "max", 150, 5, True, False),
    ("right_middle_abd", "max", 150, 5, True, False),
    ("right_ring_abd", "min", 150, 5, True, False),
    ("right_pinky_abd", "max", 150, 10, True, False),

    
    
    # Phase 4: Add more joints here as needed
    # Example:
    ("right_pinky_pip", "min", 150, 10, True, True),
    ("right_ring_pip", "min", 150, 10, True, True),
    ("right_middle_pip", "min", 150, 10, True, True),
    ("right_index_pip", "min", 150, 10, True, True),

    ("right_index_pip", "max", 150, 10, True, True),
    ("right_middle_pip", "max", 150, 10, True, True),
    ("right_ring_pip", "max", 150, 10, True, True),
    ("right_pinky_pip", "max", 150, 10, True, True),

    ("right_pinky_mcp", "min", 150, 10, True, True),
    ("right_ring_mcp", "min", 150, 10, True, True),
    ("right_middle_mcp", "min", 150, 10, True, True),
    ("right_index_mcp", "min", 150, 10, True, True),

    ("right_index_mcp", "max", 150, 10, True, True),
    ("right_middle_mcp", "max", 150, 10, True, True),
    ("right_ring_mcp", "max", 150, 10, True, True),
    ("right_pinky_mcp", "max", 150, 10, True, True),
]


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def print_joint_params_summary():
    """Print a summary of all joint parameters"""
    print("\n" + "="*70)
    print("JOINT PARAMETERS CONFIGURATION")
    print("="*70)
    
    for joint_name, params in JOINT_PARAMS.items():
        if joint_name == "_default":
            print(f"\nDefault (for unlisted joints):")
        else:
            print(f"\n{joint_name}:")
        print(f"  Max Torque Threshold: {params['max_torque_threshold']}")
        print(f"  Max Load Increase: {params['max_load_increase']}")
    
    print("\n" + "="*70)


def print_calibration_order_summary():
    """Print a summary of the calibration order"""
    print("\n" + "="*70)
    print("CALIBRATION ORDER CONFIGURATION")
    print("="*70)
    print(f"\nTotal Steps: {len(CALIBRATION_ORDER)}\n")
    
    for i, (joint_name, direction, max_torque, max_load_increase, return_to_center, set_as_middle) in enumerate(CALIBRATION_ORDER, 1):
        print(f"Step {i:2d}: {joint_name:20s} {direction:5s} | "
              f"Torque={max_torque:3d} | Load={max_load_increase:3d} | "
              f"Return={'Yes' if return_to_center else 'No':3s} | "
              f"SetMiddle={'Yes' if set_as_middle else 'No':3s}")
    
    print("\n" + "="*70)


if __name__ == "__main__":
    """Print configuration summary when run directly"""
    print("="*70)
    print("ORCA Hand Joint Configuration")
    print("="*70)
    
    print_joint_params_summary()
    print_calibration_order_summary()
