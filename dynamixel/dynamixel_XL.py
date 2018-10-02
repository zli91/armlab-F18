import dynamixel_functions as dxl

# Control table address for Dynamixel XL
#                         ADDR      ACCESS  BYTES
#EEPROM_ADDRESSES               
ADDR_MODEL_NUMBER        = 0    #   R       2
ADDR_FIRMWARE_VERSION    = 2    #   R       1
ADDR_ID                  = 3    #   RW      1
ADDR_BAUD_RATE           = 4    #   RW      1
ADDR_RETURN_DELAY        = 5    #   RW      1
ADDR_CW_LIMIT            = 6    #   RW      2
ADDR_CCW_LIMIT           = 8    #   RW      2
ADDR_CONTROL_MODE        = 11   #   RW      1
ADDR_TEMP_LIMIT          = 12   #   RW      1
ADDR_LOW_VOLTAGE_LIMIT   = 13   #   RW      1
ADDR_HIGH_VOLTAGE_LIMIT  = 14   #   RW      1
ADDR_MAX_TORQUE          = 15   #   RW      2
ADDR_STATUS_RTN          = 17   #   RW      1
ADDR_ALARM_SHUTDOWN      = 18   #   RW      1

#RAM_ADDRESSES
ADDR_TORQUE_ENABLE       = 24   #   RW      1
ADDR_LED                 = 25   #   RW      1
ADDR_D_GAIN              = 27   #   RW      1
ADDR_I_GAIN              = 28   #   RW      1
ADDR_P_GAIN              = 29   #   RW      1
ADDR_GOAL_POSITION       = 30   #   RW      2
ADDR_MOVING_SPEED        = 32   #   RW      2
ADDR_TORQUE_LIMIT        = 35   #   RW      2
ADDR_PRESENT_POSITION    = 37   #   R       2
ADDR_PRESENT_SPEED       = 39   #   R       2
ADDR_PRESENT_LOAD        = 41   #   R       2
ADDR_PRESENT_VOLTAGE     = 45   #   R       1
ADDR_PRESENT_TEMP        = 46   #   R       1
ADDR_INSTR_REG           = 47   #   R       1
ADDR_MOVING              = 49   #   R       1
ADDR_HARDWARE_ERROR      = 50   #   R       1
ADDR_PUNCH               = 51   #   RW      2

PROTOCOL                 = 2

class DXL_XL:
    def __init__(self, port, id):
        self.type = "XL"
        self.port = port
        self.id = id
        self.mode = self.get_mode()
        self.max_speed = 11.89 #rad/s

    def set_mode(self,mode):
        # 1 for wheel, 2 for joint
        if((mode != 1) & (mode != 2)):
            print('Error: Mode must be 1 or 2')
            return
        dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CONTROL_MODE, mode)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))

    def get_mode(self):
        mode = dxl.read1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CONTROL_MODE)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))
        return mode

    def set_gains(self,kp,ki,kd):
        dxl.write2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_P_GAIN, kp)
        dxl.write2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_I_GAIN, ki)
        dxl.write2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_D_GAIN, kd)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))

    def enable_torque(self):
        dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_TORQUE_ENABLE, 1)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))

    def disable_torque(self):
        dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_TORQUE_ENABLE, 0)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))

        # set_position() takes position in radians from -pi to pi
    def set_position(self, pos):
        value = self.rad2value(pos)
        dxl.write2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_GOAL_POSITION, value)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))

    # set_speed() takes -1.0 to 1.0
    def set_speed(self, speed):
        value = 0
        if(self.mode == 2):
            value = int(1023*abs(speed))
        elif(self.mode == 1):
            value = int(1023*abs(speed))
            if(speed>0.0):
                value = value + 1024
        dxl.write2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_MOVING_SPEED, value)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))

    def set_torque_limit(self, torque):
        dxl.write2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_TORQUE_LIMIT, int(torque * 1023))
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))

    def get_position(self):
        pos = dxl.read2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_PRESENT_POSITION)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))
        return self.value2rad(pos)

    def get_speed(self):
        speed = dxl.read2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_PRESENT_SPEED)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))
        return (speed/1023.0)-1.0

    def get_load(self):
        load = dxl.read2ByteTxRx(self.port, PROTOCOL, self.id, ADDR_PRESENT_LOAD)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))
        return (load/1023.0)-1.0

    def get_temp(self):
        temp = dxl.read1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_PRESENT_TEMP)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))
        return temp

    def get_voltage(self):
        voltage = dxl.read1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_PRESENT_VOLTAGE)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))
        return voltage/10.0

    def is_moving(self):
        moving = dxl.read1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_MOVING)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))
        return moving

    def get_error_status(self):
        error_status = dxl.read1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_HARDWARE_ERROR)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))
        return error_status

    def deg2value(self, angle):
        return int(1023*(angle + 150.0)/300)

    def value2deg(self, value):
        return (300.0*value/1023.0)-150.0

    def rad2value(self, angle):
        return int(1023*(angle + 2.617993878)/5.235987756)

    def value2rad(self, value):
        return ((5.235987756*value/1023.0)-2.617993878)
