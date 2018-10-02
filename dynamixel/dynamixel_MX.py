import dynamixel_functions as dxl

# Control table address for Dynamixel MX
#EEPROM_ADDRESSES
ADDR_MODEL_NUMBER        = 0
ADDR_FIRMWARE_VERSION    = 2
ADDR_ID                  = 3
ADDR_BAUD_RATE           = 4
ADDR_RETURN_DELAY        = 5
ADDR_CW_LIMIT            = 6
ADDR_CCW_LIMIT           = 8
ADDR_TEMP_LIMIT          = 11
ADDR_LOW_VOLTAGE_LIMIT   = 12
ADDR_HIGH_VOLTAGE_LIMIT  = 13
ADDR_MAX_TORQUE          = 14
ADDR_STATUS_RTN          = 16
ADDR_ALARM_LED           = 17
ADDR_ALARM_SHUTDOWN      = 18

#RAM_ADDRESSES
ADDR_TORQUE_ENABLE       = 24
ADDR_LED                 = 25
ADDR_D_GAIN              = 26
ADDR_I_GAIN              = 27
ADDR_P_GAIN              = 28
ADDR_GOAL_POSITION       = 30
ADDR_MOVING_SPEED        = 32
ADDR_TORQUE_LIMIT        = 34
ADDR_PRESENT_POSITION    = 36
ADDR_PRESENT_SPEED       = 38
ADDR_PRESENT_LOAD        = 40
ADDR_PRESENT_VOLTAGE     = 42
ADDR_PRESENT_TEMP        = 43
ADDR_INSTR_REG           = 44
ADDR_MOVING              = 46
ADDR_LOCK                = 47
ADDR_PUNCH               = 48

PROTOCOL                 = 1

class DXL_MX:
    def __init__(self, port, id):
        self.type = "MX"
        self.port = port
        self.id = id
        self.mode = self.get_mode()
        self.max_speed = 12.2595 #rad/s

    def set_mode(self,mode):
        # 1 for wheel, 2 for joint
        if((mode != 1) & (mode != 2)):
            print('Error: Mode must be 1 or 2')
            return       
        if(mode == 1):
            dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CW_LIMIT, 0)
            dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CCW_LIMIT, 0)
        if(mode == 2):
            dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CW_LIMIT, 0)
            dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CCW_LIMIT, 4095)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))
        self.mode = mode

    def get_mode(self):
        mode=0
        cw_lim = dxl.read1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CW_LIMIT)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))
        ccw_lim = dxl.read1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_CCW_LIMIT)
        dcomm_result = dxl.getLastTxRxResult(self.port, PROTOCOL)
        derror = dxl.getLastRxPacketError(self.port, PROTOCOL)
        if dcomm_result != 0:
            print(dxl.getTxRxResult(PROTOCOL, dcomm_result))
        elif derror != 0:
            print(dxl.getRxPacketError(PROTOCOL, derror))
        if((cw_lim == 0)&(ccw_lim == 0)):
            mode = 1
        else:
            mode = 2
        return mode

    def set_gains(self,kp,ki,kd):
        dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_P_GAIN, kp)
        dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_I_GAIN, ki)
        dxl.write1ByteTxRx(self.port, PROTOCOL, self.id, ADDR_D_GAIN, kd)
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

    def deg2value(self, angle):
        return int(4095.0*(angle+180)/360.0)

    def value2deg(self, value):
        return (360.0*(value-2048)/4095)

    def rad2value(self, angle):
        return int(4095*(angle + 3.141592654)/6.283185307)

    def value2rad(self, value):
        return ((6.283185307*value/4095.0)-3.141592654)
