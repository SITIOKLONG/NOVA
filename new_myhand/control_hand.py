import os
import sys

from scservo_sdk import *
import yaml
import time

def read_servo_info():
    for servo_id in config['servo_to_joint_map']:
        joint_name = config['servo_to_joint_map'][servo_id]
        
        # Read min angle limit
        min_limit, scs_comm_result, scs_error = packetHandler.read2ByteTxRx(portHandler, servo_id, ADDR_SCS_MIN_ANGLE_LIMIT)
        if scs_comm_result != COMM_SUCCESS:
            print("[ID:%03d] %s Min limit read failed: %s" % (servo_id, joint_name, packetHandler.getTxRxResult(scs_comm_result)))
            continue
        elif scs_error != 0:
            print("[ID:%03d] %s Min limit read error: %s" % (servo_id, joint_name, packetHandler.getRxPacketError(scs_error)))
            continue
        
        # Read max angle limit
        max_limit, scs_comm_result, scs_error = packetHandler.read2ByteTxRx(portHandler, servo_id, ADDR_SCS_MAX_ANGLE_LIMIT)
        if scs_comm_result != COMM_SUCCESS:
            print("[ID:%03d] %s Max limit read failed: %s" % (servo_id, joint_name, packetHandler.getTxRxResult(scs_comm_result)))
            continue
        elif scs_error != 0:
            print("[ID:%03d] %s Max limit read error: %s" % (servo_id, joint_name, packetHandler.getRxPacketError(scs_error)))
            continue
        
        # Store in SERVO_RANGE dictionary
        SERVO_RANGE[servo_id] = {'min': min_limit, 'max': max_limit, 'joint': joint_name}
        # print(SERVO_RANGE)

def move_to_idle():
    """ 
    wrist to center

    pinky abd to left
    ring abd to left
    middle abd to left
    index abd to left
    index abd to center
    middle abd to center
    ring abd to center
    pinky abd to center

    pinky mcp to center
    ring mcp to center
    middle mcp to center
    index mcp to center
    """
    move_joint('right_wrist', 0.5)
    transmit_joint_action()

    move_joint('right_thumb_abd',0.1)
    move_joint('right_thumb_mcp',0.1)
    move_joint('right_thumb_pip',0.1)
    move_joint('right_thumb_dip',0.1)
    transmit_joint_action()
    time.sleep(0.5)

    move_joint('right_pinky_abd',0.0)
    transmit_joint_action()
    time.sleep(0.5)
    move_joint('right_ring_abd',1.0) # inverted value
    transmit_joint_action()
    time.sleep(0.5)
    move_joint('right_middle_abd',0.0)
    transmit_joint_action()
    time.sleep(0.5)
    move_joint('right_index_abd',0.0)
    transmit_joint_action()
    time.sleep(0.5)

    move_joint('right_index_abd',0.5)
    transmit_joint_action()
    time.sleep(0.5)
    move_joint('right_middle_abd',0.5)
    transmit_joint_action()
    time.sleep(0.5)
    move_joint('right_ring_abd',0.5)
    transmit_joint_action()
    time.sleep(0.5)
    move_joint('right_pinky_abd',0.5)
    transmit_joint_action()
    time.sleep(0.5)

    move_joint('right_pinky_pip',0.5)
    transmit_joint_action()
    time.sleep(0.5)
    move_joint('right_ring_pip',0.5)
    transmit_joint_action()
    time.sleep(0.5)
    move_joint('right_middle_pip',0.5)
    transmit_joint_action()
    time.sleep(0.5)
    move_joint('right_index_pip',0.5)
    transmit_joint_action()
    time.sleep(0.5)

    move_joint('right_pinky_mcp',0.5)
    transmit_joint_action()
    time.sleep(0.5)
    move_joint('right_ring_mcp',0.5)
    transmit_joint_action()
    time.sleep(0.5)
    move_joint('right_middle_mcp',0.5)
    transmit_joint_action()
    time.sleep(0.5)
    move_joint('right_index_mcp',0.5)
    transmit_joint_action()
    time.sleep(0.5)

    move_joint('right_thumb_abd',0.5)
    move_joint('right_thumb_mcp',0.5)
    move_joint('right_thumb_pip',0.5)
    move_joint('right_thumb_dip',0.5)
    transmit_joint_action()
    time.sleep(0.5)

    return True, None

def move_to_fuck():
    move_joint('right_thumb_abd',0.1)
    move_joint('right_thumb_mcp',0.1)
    move_joint('right_thumb_pip',0.1)
    move_joint('right_thumb_dip',0.1)
    transmit_joint_action()
    time.sleep(1.0)

    move_joint('right_pinky_abd',0.5)
    move_joint('right_ring_abd',0.5)
    move_joint('right_middle_abd',0.5)
    move_joint('right_index_abd',0.5)
    move_joint('right_pinky_mcp',1.0)
    move_joint('right_ring_mcp',1.0)
    move_joint('right_index_mcp',1.0)
    move_joint('right_pinky_pip',0.9) #dangerous
    move_joint('right_ring_pip',1.0)
    move_joint('right_index_pip',1.0)

    move_joint('right_middle_mcp',0.2)
    move_joint('right_middle_pip',0.2)
    transmit_joint_action()
    time.sleep(1.0)

    move_joint('right_thumb_abd',0.8)
    move_joint('right_thumb_mcp',0.5)
    move_joint('right_thumb_pip',0.7)
    move_joint('right_thumb_dip',0.85)

    
    transmit_joint_action()
    time.sleep(0.5)

def move_to_catching():
    move_joint('right_thumb_abd',0.1)
    move_joint('right_thumb_mcp',0.1)
    move_joint('right_thumb_pip',0.1)
    move_joint('right_thumb_dip',0.1)
    transmit_joint_action()
    time.sleep(0.5)

    move_joint('right_pinky_mcp',0.1)
    move_joint('right_ring_mcp',0.1)
    move_joint('right_middle_mcp',0.1)
    move_joint('right_index_mcp',0.1)
    move_joint('right_pinky_pip',0.1)
    move_joint('right_ring_pip',0.1)
    move_joint('right_middle_pip',0.1)
    move_joint('right_index_pip',0.1)
    transmit_joint_action()
    time.sleep(1.0)

    move_joint('right_pinky_abd',0.8)
    move_joint('right_ring_abd',0.4) #inverted value
    move_joint('right_middle_abd',0.5)
    move_joint('right_index_abd',0.3)


    move_joint('right_pinky_mcp',0.8)
    move_joint('right_ring_mcp',1.0)
    move_joint('right_middle_mcp',0.85)
    move_joint('right_index_mcp',0.75)
    move_joint('right_pinky_pip',0.45)
    move_joint('right_ring_pip',0.45)
    move_joint('right_middle_pip',0.50)
    move_joint('right_index_pip',0.55)
    transmit_joint_action()

    


    # wait until user presses Enter only
    try:
        # set terminal to raw mode to read single keypresses (so Esc can be detected)
        tty.setraw(fd)
        print("Press Enter to continue, Esc to exit")
        while True:
            ch = sys.stdin.read(1)
            if ch == '\x1b':  # ESC
                # restore terminal and exit program
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                sys.exit(0)
            if ch in ('\r', '\n'):
                break
        print()
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        # ensure terminal settings are restored
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    move_joint('right_thumb_abd',0.5)
    move_joint('right_thumb_mcp',0.5)
    move_joint('right_thumb_pip',0.5)
    move_joint('right_thumb_dip',0.8)
    transmit_joint_action()
    time.sleep(0.5)

def move_to_safety():
    move_joint('right_thumb_abd',0.1)
    move_joint('right_thumb_mcp',0.1)
    move_joint('right_thumb_pip',0.1)
    move_joint('right_thumb_dip',0.1)
    transmit_joint_action()
    time.sleep(1.0)

    move_joint('right_pinky_abd',0.5)
    move_joint('right_ring_abd',0.5)
    move_joint('right_middle_abd',0.5)
    move_joint('right_index_abd',0.5)
    move_joint('right_pinky_mcp',1.0)
    move_joint('right_ring_mcp',1.0)
    move_joint('right_middle_mcp',1.0)
    move_joint('right_index_mcp',1.0)
    move_joint('right_pinky_pip',0.9) #dangerous
    move_joint('right_ring_pip',1.0)
    move_joint('right_middle_pip',1.0)
    move_joint('right_index_pip',1.0)

    transmit_joint_action()
    time.sleep(1.0)

    move_joint('right_thumb_abd',0.8)
    move_joint('right_thumb_mcp',0.5)
    move_joint('right_thumb_pip',0.8)
    move_joint('right_thumb_dip',0.5)

    
    transmit_joint_action()
    time.sleep(0.5)



def self_check():
    move_joint('right_pinky_abd',0.0)
    transmit_joint_action()
    time.sleep(0.5)


def move_joint(joint_name, position):
    '''
    joint_name: joints name, check in config.yaml servo_to_joint_map
    position: normalize 0-1, check direction min/max in config.yaml table_of_joints
    '''
    for sid, info in SERVO_RANGE.items():
        joint = info.get('joint', '') if isinstance(info, dict) else ''
        if joint_name.lower() == joint.lower():
            servo_id = sid
            # get limits
            min_limit = int(info['min'])
            max_limit = int(info['max'])
            
            p = max(0.0, min(1.0, position))

            # interpolate between min and max
            target = int(min_limit + p * (max_limit - min_limit))

            # Prepare parameter (2 bytes: low byte, high byte)
            param_goal_position = [SCS_LOBYTE(target), SCS_HIBYTE(target)]

            # Add parameter to sync write
            if not groupSyncWrite.addParam(servo_id, bytes(param_goal_position)):
                return False, f"Failed to add parameter for servo {servo_id}"

    return False, f"Joint '{joint_name}' not found"
    

def transmit_joint_action():
    # Transmit sync write packet
    comm_result = groupSyncWrite.txPacket()
    # Clear parameter storage
    groupSyncWrite.clearParam()
    if comm_result != COMM_SUCCESS:
        return False, f"Communication error: {packetHandler.getTxRxResult(comm_result)}"

if __name__ == "__main__":

    # Control table addresses
    ADDR_STS_PRESENT_POSITION  = 56
    ADDR_SCS_MIN_ANGLE_LIMIT   = 9
    ADDR_SCS_MAX_ANGLE_LIMIT   = 11
    # Control table address
    ADDR_SCS_TORQUE_ENABLE     = 40
    ADDR_STS_GOAL_ACC          = 41
    ADDR_STS_GOAL_POSITION     = 42
    ADDR_STS_GOAL_SPEED        = 46

    SCS_MINIMUM_POSITION_VALUE  = 0               # SCServo will rotate between this value
    SCS_MAXIMUM_POSITION_VALUE  = 4095              # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
    SCS_MOVING_STATUS_THRESHOLD = 20                # SCServo moving status threshold
    SCS_MOVING_SPEED            = 0                 # SCServo moving speed
    SCS_MOVING_ACC              = 0                 # SCServo moving acc
    protocol_end                = 0                 # SCServo bit end(STS/SMS=0, SCS=1)


    # Servo settings
    BAUDRATE = 1000000
    DEVICENAME = '/dev/ttyACM0'
    protocol_end = 0  # STS/SMS=0, SCS=1


    SERVO_RANGE = {} # a dict saving min/max value of servo

    with open('config.yaml', 'r') as f:
        config = yaml.safe_load(f)

    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Get methods and members of Protocol
    packetHandler = PacketHandler(protocol_end)

    # Initialize GroupSyncWrite instance
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_STS_GOAL_POSITION, 2)

    # Initialize GroupSyncRead instace for Present Position
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_STS_PRESENT_POSITION, 4)

        
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        quit()


    read_servo_info()

    # move_to_idle() # hardcoded


    # self_check() # fack
    # move_to_fuck()
    # move_to_catching()
    move_to_safety()



    # while 1:
    #     ...