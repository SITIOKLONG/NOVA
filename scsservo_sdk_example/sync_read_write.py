#!/usr/bin/env python
#
# *********     Sync Read and Sync Write Example      *********
#
#
# Available SCServo model on this example : All models using Protocol SCS
# This example is tested with a SCServo(STS/SMS), and an URT
# Be sure that SCServo(STS/SMS) properties are already set as %% ID : 1 / Baudnum : 6 (Baudrate : 1000000)
#

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from scservo_sdk import *                    # Uses SCServo SDK library

# Control table address
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_STS_GOAL_ACC          = 41
ADDR_STS_GOAL_POSITION     = 42
ADDR_STS_GOAL_SPEED        = 46
ADDR_STS_PRESENT_POSITION  = 56
ADDR_SCS_MIN_ANGLE_LIMIT   = 9      # Min angle limit (2 bytes)
ADDR_SCS_MAX_ANGLE_LIMIT   = 11     # Max angle limit (2 bytes)

# Default setting
SERVO_IDS                   = list(range(1, 18))  # SCServo IDs: 1-17
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
DEVICENAME                  = '/dev/ttyACM0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_MINIMUM_POSITION_VALUE  = 0               # SCServo will rotate between this value
SCS_MAXIMUM_POSITION_VALUE  = 4095              # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
SCS_MOVING_STATUS_THRESHOLD = 20                # SCServo moving status threshold
SCS_MOVING_SPEED            = 0                 # SCServo moving speed
SCS_MOVING_ACC              = 0                 # SCServo moving acc
protocol_end                = 0                 # SCServo bit end(STS/SMS=0, SCS=1)

index = 0
scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]         # Goal position

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
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

def read_servo_range():
    
# Read min and max position limits for all servos
print("\n=== Reading Min/Max Position Limits for All Servos ===")
for servo_id in SERVO_IDS:
    # Read min angle limit
    min_limit, scs_comm_result, scs_error = packetHandler.read2ByteTxRx(portHandler, servo_id, ADDR_SCS_MIN_ANGLE_LIMIT)
    if scs_comm_result != COMM_SUCCESS:
        print("[ID:%03d] Min limit read failed: %s" % (servo_id, packetHandler.getTxRxResult(scs_comm_result)))
        continue
    elif scs_error != 0:
        print("[ID:%03d] Min limit read error: %s" % (servo_id, packetHandler.getRxPacketError(scs_error)))
        continue
    
    # Read max angle limit
    max_limit, scs_comm_result, scs_error = packetHandler.read2ByteTxRx(portHandler, servo_id, ADDR_SCS_MAX_ANGLE_LIMIT)
    if scs_comm_result != COMM_SUCCESS:
        print("[ID:%03d] Max limit read failed: %s" % (servo_id, packetHandler.getTxRxResult(scs_comm_result)))
        continue
    elif scs_error != 0:
        print("[ID:%03d] Max limit read error: %s" % (servo_id, packetHandler.getRxPacketError(scs_error)))
        continue
    
    print("[ID:%03d] Min Limit: %04d | Max Limit: %04d | Range: %04d" % 
          (servo_id, min_limit, max_limit, max_limit - min_limit))

print("\n=== Starting Position Monitoring ===\n")

# Add parameter storage for all servos
for servo_id in SERVO_IDS:
    scs_addparam_result = groupSyncRead.addParam(servo_id)
    if scs_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % servo_id)
        quit()

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # Allocate goal position value into byte array
    param_goal_position = [SCS_LOBYTE(scs_goal_position[index]), SCS_HIBYTE(scs_goal_position[index])]

    # Add all servo goal position values to the Syncwrite parameter storage
    for servo_id in SERVO_IDS:
        scs_addparam_result = groupSyncWrite.addParam(servo_id, param_goal_position)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % servo_id)
            quit()

    # Syncwrite goal position
    scs_comm_result = groupSyncWrite.txPacket()
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    while 1:
        # Syncread present position
        scs_comm_result = groupSyncRead.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))

        # Read data for all servos
        servo_data = []
        for servo_id in SERVO_IDS:
            scs_getdata_result = groupSyncRead.isAvailable(servo_id, ADDR_STS_PRESENT_POSITION, 4)
            if scs_getdata_result == True:
                present_position_speed = groupSyncRead.getData(servo_id, ADDR_STS_PRESENT_POSITION, 4)
                present_position = SCS_LOWORD(present_position_speed)
                present_speed = SCS_HIWORD(present_position_speed)
                servo_data.append((servo_id, present_position, present_speed))
                print("[ID:%03d] PresPos:%04d PresSpd:%04d" % 
                      (servo_id, present_position, SCS_TOHOST(present_speed, 15)), end="\t")
            else:
                print("[ID:%03d] groupSyncRead getdata failed" % servo_id, end="\t")
        
        print()  # New line after printing all servos
        break  # Exit after one read since writing is commented out

# Clear syncread parameter storage
groupSyncRead.clearParam()

# Disable torque for all servos
for servo_id in SERVO_IDS:
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_SCS_TORQUE_ENABLE, 0)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

# Close port
portHandler.closePort()