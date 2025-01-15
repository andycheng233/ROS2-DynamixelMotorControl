import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from dynamixel_sdk import *
import sys

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24                 # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_SPEED              = 32
ADDR_MX_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
#DXL_ID                      = 5                 # Dynamixel ID : 1
BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = Int32(data=1)          # Value for enabling the torque
TORQUE_DISABLE              = Int32(data=0)          # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0                 # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023              # and this value 
DXL_MINIMUM_SPEED_VALUE  = 0                    # Dynamixel will rotate between this value
DXL_MAXIMUM_SPEED_VALUE  = 1023                 # and this value 

class ControlMotor(Node):
    def __init__(self, motor_id):
        super().__init__(f'control_motor_{motor_id}')

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.motor_id = motor_id
        #self.motor_id = 5

        #open port
        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open the port!")
            return

        #set up baudrate
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set the baud rate!")
            return

        if not self.set_torque(TORQUE_ENABLE):
            return

        self.torque_subscription = self.create_subscription(
            Int32,
            f'motor_{motor_id}/set_torque',
            self.set_torque,
            10
        )

        self.position_subscription = self.create_subscription(
            Int32, 
            f'motor_{motor_id}/set_position',
            self.set_position,
            10
        )

        self.speed_subscription = self.create_subscription(
            Int32,
            f'motor_{motor_id}/set_speed',
            self.set_speed,
            10
        )

        print(f"Successfully running [Motor {motor_id}]!")

    def set_torque(self, msg):
        enable = msg.data

        if enable not in [0,1]:
            print(f"Motor {self.motor_id}: Invalid torque value. Must be 0 or 1.")
            return 0

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.motor_id, ADDR_MX_TORQUE_ENABLE, enable)
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Motor {self.motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return 0

        elif dxl_error != 0:
            print(f"Motor {self.motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
            return 0

        return 1
                
    def set_position(self, msg):
        goal_position = msg.data
        if goal_position > DXL_MAXIMUM_POSITION_VALUE or goal_position < DXL_MINIMUM_POSITION_VALUE:
            print(f"Motor {self.motor_id}: Out of position range!")
            return 0
        
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.motor_id, ADDR_MX_GOAL_POSITION, goal_position)
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Motor {self.motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return 0

        elif dxl_error != 0:
            print(f"Motor {self.motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
            return 0

    def set_speed(self, msg):
        speed = msg.data
        if speed > DXL_MAXIMUM_SPEED_VALUE or speed < DXL_MINIMUM_SPEED_VALUE:
            print(f"Motor {self.motor_id}: Out of speed range!")
            return 0

        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.motor_id, ADDR_MX_SPEED, speed)
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Motor {self.motor_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return 0

        elif dxl_error != 0:
            print(f"Motor {self.motor_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
            return 0

    def shutdown(self):
        self.set_torque(TORQUE_DISABLE)
        self.portHandler.closePort()
        print(f"Shutdown [Motor {self.motor_id}]!")

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 2:
        print("Correct Usage: ros2 run run_motors control_motor <motor_id>!")
        return
    motor_id = int(sys.argv[1])
    node = ControlMotor(motor_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()

if __name__ == '__main__':
    main()
        
