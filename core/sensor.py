import smbus, logging, pdb
from threading import Lock
from sys import stdout as sys_stdout
from bluedot.btcomm import BluetoothServer
from signal import pause

# logging.basicConfig(
#     level=logging.DEBUG,
#     format='%(asctime)s - %(levelname)s - %(message)s',
#     handlers=[
#         # logging.FileHandler('test.log'),
#         logging.StreamHandler(sys_stdout)
#     ]
# )

class MPU6050(smbus.SMBus):
    ''' Designed specifically for TrackNRoll project requirments '''
    
    # Registers
    PWR_MGT_1      = 0x6B
    PWR_MGT_2      = 0x6C
    SMPLRT_DIV     = 0x19
    CONFIG         = 0x1A
    INT_PIN_CFG    = 0x37
    INT_ENABLE     = 0x38
    INT_STATUS     = 0x3A
    
    def __init__(self, bus, address=0x68, STDBY_ACCEL=True, INT_enabled=False, INT_pin=None):
        super(MPU6050, self).__init__(bus=bus)
        self.DEV_ADDR = address
        # Make sure only one thread can access the bus at any given time.
        self.bus_lock = Lock()
        self.data_ready_lock = Lock()

        # Initialize MPU6050
        self.initialize( 
            STDBY_ACCEL=STDBY_ACCEL,
            INT_enabled=INT_enabled,
            INT_pin=INT_pin
        )

        self.data_ready = False

    def initialize(self, STDBY_ACCEL=True, INT_enabled=False, INT_pin=None):
        logging.debug(msg='Initializing {} ... '.format(self.__class__.__name__))
        # Wake Chip and CLKSEL = PLL X axis gyro ref
        self.i2c_write(self.DEV_ADDR, MPU6050.PWR_MGT_1, 1)

        # Set Sample Rate to 1000;  Fs/(1+SMPLRT_DIV)
        # Sensor reading will be available every 100 ms
        self.i2c_write(self.DEV_ADDR, MPU6050.CONFIG, 6)
        self.i2c_write(self.DEV_ADDR, MPU6050.SMPLRT_DIV, 99)

        if STDBY_ACCEL:
            # Put accelerometer axes on standby mode
            logging.debug('Putting Accelerometer on standby mode ... ')
            self.i2c_write(self.DEV_ADDR, MPU6050.PWR_MGT_2, 56)

        if INT_enabled and INT_pin:
            # INT: Active Low, Open Drain, Cleared by reading INT_STATUS
            logging.debug('Enabling DATA_RDY_EN interrupt ... ')
            self.i2c_write(self.DEV_ADDR, MPU6050.INT_PIN_CFG, 224)
            self.INT_pin = INT_pin
            self.INT_pin.when_pressed = self._data_ready_callback
            self.INT_enabled = True

            # Clear INT_STATUS Register
            INT_STATUS = self.i2c_read(self.DEV_ADDR, MPU6050.INT_STATUS)
            logging.debug(msg='INT_STATUS register value cleared {}.'.format(bin(INT_STATUS)))

        logging.debug(msg='{} initialization done'.format(self.__class__.__name__))

    @property
    def INT_pin(self):
        return self._INT_pin
    
    @INT_pin.setter
    def INT_pin(self, pin):
        # pin: Raspberry Pi GPIO pin number (BCM) where the sensor's INT
        # pin is connected to. Make sure to use BCM pin numbering.
        if isinstance(pin.pin.number, int) and pin.pin.number > 0 and pin.pin.number < 28:
            self._INT_pin = pin

    @property
    def INT_enabled(self):
        # Method to set or clear the DATA_RDY_EN bit on the INT_ENABLE register.
        # When True, MPU6050 INT pin will fire an interrupt signal whenever a
        # new data is written on the sensors data registers.
        DATA_RDY_EN = self.i2c_read(self.DEV_ADDR, MPU6050.INT_ENABLE) & 0x01
        return bool(DATA_RDY_EN)

    @INT_enabled.setter
    def INT_enabled(self, value):
        # See getter comments
        value = bool(value)
        INT_ENABLE = self.i2c_read(self.DEV_ADDR, MPU6050.INT_ENABLE) | value
        self.i2c_write(self.DEV_ADDR, MPU6050.INT_ENABLE, INT_ENABLE)
        self.INT_clear()
    
    def INT_clear(self):
        # Clears the INT_STATUS register, hence clearing the MPU6050 INT
        # pin from its active_state. Need to call this method everytime
        # a data was red from the sensor data registers to return the
        # INT pin to its idle state. Else INT pin will never be released
        # from its active state.
        self.i2c_read(self.DEV_ADDR, MPU6050.INT_STATUS)

    def _data_ready_callback(self):
        # Serves as the callback function of INT_pin.when_pressed event
        # which serves as workaround to set the self.data_ready to True
        # This method is needed since the when_pressed property of
        # Button object only accepts function with no arguments.
        # INT_pin.when_pressed = self.data_ready is not valid since
        # the self.data_ready property requires a boolean argument.
        self.data_ready = True

    @property
    def data_ready(self):
        # Used in conjunction with the INT_pin.when_pressed, the value
        # is set to True inside the data_ready_callback function caused by
        # the DATA_RDY_EN interrupt. Return this to False after reading
        # the data from sensor registers and clearing the INT_STATUS
        # register by calling the INT_clear method.
        with self.data_ready_lock:
            return self._data_ready

    @data_ready.setter
    def data_ready(self, value):
        # See getter's comment
        with self.data_ready_lock:
            self._data_ready = bool(value)

    def i2c_write(self, dev_addr, reg_addr, data):
        # Used in conjunction with bus_lock to ensure only one
        # thread can access the i2c bus at instance.
        with self.bus_lock:
            super().write_byte_data(dev_addr, reg_addr, data)

    def i2c_read(self, dev_addr, reg_addr):
        with self.bus_lock:
            return super().read_byte_data(dev_addr, reg_addr)

    def reset(self):
        # Resets the chip internal registers to their default values
        logging.debug(msg='Resetting MPU6050 ...')
        reg = self.i2c_read(self.DEV_ADDR, MPU6050.PWR_MGT_1) | (1 << 6)
        self.i2c_write(self.DEV_ADDR, MPU6050.PWR_MGT_1, reg)
        logging.debug(msg='MPU6050 reset done.')

    @property
    def sleep(self):
        PWR_MGT_1 = self.i2c_read(self.DEV_ADDR, MPU6050.PWR_MGT_1)
        PWR_MGT_1 &= (1 << 6)
        return bool(PWR_MGT_1)
    
    @sleep.setter
    def sleep(self, value):
        # value: bool
        value = bool(value)
        PWR_MGT_1 = self.i2c_read(self.DEV_ADDR, MPU6050.PWR_MGT_1)
        if value:
            PWR_MGT_1 |= (1 << 6) # Set PWR_MGT_1 bit 6
        else:
            PWR_MGT_1 |= (0 << 6) # Clear PWR_MGT_1 bit 6
        self.i2c_write(self.DEV_ADDR, MPU6050.PWR_MGT_1, PWR_MGT_1)

    @property
    def gyro_fsr(self):
        # Gyro test modes are discarded
        fsr = self.i2c_read(self.DEV_ADDR, 0x1B)
        fsr &= 0b00011000 # Discard other bits, only FSR
        return fsr >> 3
    
    @gyro_fsr.setter
    def gyro_fsr(self, value):
        # value: int from 0 to 3
        if value < 0:
            fsr = 0
        elif value > 3:
            fsr = 3
        else:
            fsr = value
        fsr *= 8 # place value on FSR bit position
        self.i2c_write(self.DEV_ADDR, 0x1B, fsr)

    def gyro_read(self, axis='', raw=True):
        
        # Recursion
        if not axis:
            read_out = list()
            for _ in ('x', 'y', 'z'):
                read_out.append(self.gyro_read(axis=_))
            return tuple(read_out)

        axes = dict(
            x = 0x43,
            y = 0x45,
            z = 0x47
        )
        axis = axes[axis.lower()]

        high_byte = self.i2c_read(self.DEV_ADDR, axis)
        low_byte  = self.i2c_read(self.DEV_ADDR, axis+1)

        # Concatenate high_byte and low_byte
        value = high_byte << 8   # Make it 16 bit. left-shift high_byte by 8 bits
        value = value | low_byte # Combine high_byte and low_byte

        # Get the signed value
        if value > pow(2, 16)/2:
            value = value - pow(2, 16)

        return value

    @property
    def temp_enabled(self):
        PWR_MGT_1 = self.i2c_read(self.DEV_ADDR, MPU6050.PWR_MGT_1)
        TEMP_DIS = (PWR_MGT_1 & (1 << 3)) >> 3
        return not bool(TEMP_DIS)

    @temp_enabled.setter
    def temp_enabled(self, value):
        logging.debug(msg='Setting TEMP_SENSOR enable to {} ...'.format(value))
        value = (not bool(value)) << 3
        value |= self.i2c_read(self.DEV_ADDR, MPU6050.PWR_MGT_1)
        self.i2c_write(self.DEV_ADDR, MPU6050.PWR_MGT_1, value)
        logging.debug(msg='TEMP_SENSOR settings done ...')

    def temp_read(self):
        logging.debug(msg='Reading TEMP_OUT registers ... ')
        high_byte = self.i2c_read(self.DEV_ADDR, 0x41)
        low_byte  = self.i2c_read(self.DEV_ADDR, 0x42)
        
        temp = high_byte << 8
        temp |= low_byte
        if temp > pow(2, 16)/2:
            temp -= pow(2, 16)

        temp = (temp/340) + 36.53
        return round(temp, 2)

""" ======================== TEST CODES ================================ """

if __name__ == '__main__':
    pass