import threading, logging, pdb
from core.sensor import MPU6050
from sys import stdout as sys_stdout
from time import sleep as delay
from time import time as now
from core.module import TimeoutExpired, CommandError

# logging.basicConfig(
#     level=logging.DEBUG,
#     format='%(asctime)s - %(levelname)s - %(message)s',
#     handlers=[
#         # logging.FileHandler('test.log'),
#         logging.StreamHandler(sys_stdout)
#     ]
# )

def check_passw(pw, pw_hashed):
    # compares sms provided pw with hashed pw. returns
    # True if matched, false otherwise
    from bcrypt import checkpw 
    return checkpw(pw.encode(), pw_hashed.encode())
    
class ThreadMonitor(threading.Thread):
    def __init__(self, *args, **kwargs):
        # Get all running Threads
        self.running_threads = set(threading.enumerate())
        self.rt_lock = threading.Lock()
        
    def add_thread(self, thread):
        # Add a thread on watchlist
        with self.rt_lock:
            self.running_threads.add(thread)

    def remove_thread(self, thread):
        # Remove a thread on watchlist
        with self.rt_lock:
            self.running_threads.remove(thread)

    def run(self):
        # 
        while True:
            current_running_threads = threading.enumerate()
            
            delay(1)


class Command():
    ''' Parse and authenticate an sms command '''

    cmd_map = dict()

    def __init__(self, sms, pw_hashed):
        logging.debug('Parsing sms ... ')
        self.parse(sms, pw_hashed)
        logging.debug('Command is valid: {}'.format(self.is_valid))

    def parse(self, sms, pw_hashed):
        '''Parse command from the sms and set the command 
        attributes from parsed command.
        Note:
            sms format = (
                'index', 'stat', 'number', 
                'date', 'time', 'body'
            )
        '''
        # Check if sms body is a valid command
        body = sms[len(sms)-1]  # body is the last element
        body = body.split(' ')  # split pw, cmd, arg0, arg1, ... , argn
        # A valid command must contain atleast a pw and cmd
        if len(body) < 2:       
            self.is_valid = False
            return
        
        self.command = body[1]
        if self.command not in Command.cmd_map.keys():
            self.is_valid = False
            return
        
        pw = body[0]
        if not check_passw(pw, pw_hashed):
            logging.error('Wrong password: {}'.format(pw))
            self.is_valid = False
            return
        else:
            logging.info('Correct password!')

        # Parse command arguments
        if len(body) > 2:
            self.kwargs = tuple(body[2:len(body)])
        else:
            self.kwargs = tuple()

        self.is_valid = True

    @property
    def is_valid(self):
        return self._is_valid

    @is_valid.setter
    def is_valid(self, val):
        self._is_valid = bool(val)

    @property
    def command(self):
        return self._command
    
    @command.setter
    def command(self, cmd):
        self._command = cmd

    @property
    def kwargs(self):
        return self._kwargs

    @kwargs.setter
    def kwargs(self, kwargs):
        ''' kwargs: tuple of string key-value pairs. e.g ("sms=True", "iot=False")''' 
        if not kwargs:
            self._kwargs = dict()
            return

        _filter = lambda kwarg: '=' in kwarg and '' not in kwarg.split('=')
        kwargs = [kwarg for kwarg in kwargs if _filter(kwarg)]
        # Convert tuple of string key-value pairs to keyword arguments
        self._kwargs = dict(      # SMS provided args
            [kv.split('=') for kv in kwargs]
        )
        for k,v in self._kwargs.items():  # Convert string values like
            try:
                self._kwargs[k] = eval(v)     # 'True' to True, 'False' to False
            except NameError:
                pass    # skip non-evaluatable like string args

    def execute(self):
        # Run the command
        logging.debug(
            msg='Executing {} command ...'.format(self.command)
        )
        _exe = Command.cmd_map[self.command]
        _tstart = now()
        _exe(self.kwargs)
        _exec_time = now() - _tstart
        logging.debug(
            msg='Execution finished! Exec time: {} s'.format(_exec_time)
        )


class Vehicle():
    ''' Models the vehicle where tracknroll was installed '''
    
    def __init__(self, module, iot_server, sensor, *args, **kwargs):
        self.module = module
        self.sensor = sensor

        self.iot_server = iot_server
        self.ms_thread      = None  # thread for movement sensing
        self.iot_thread     = None  # thread for iot
        self.park_loc       = None

        self.movement_sense = False
        self.iot_feed = False

    def get_location(self, gsmloc_backup=True):
        ''' Use GPS provided data if available. If gsmloc_backup=True, 
        use Network provided if GPS is not available.
        gsmloc_backup: bool
        return: GPSData (See module.py) | False if no source available
        '''
        # pdb.set_trace()
        logging.debug(msg='Getting location ... ')
        gps_stat = self.module.gps_stat()

        if gps_stat == 'Location Unknown':
            logging.error('GPS is OFF. Turning ON GPS engine ...')
            self.module.gps_pwr = 1
            gps_stat = self.module.gps_stat()

        if gps_stat == 'Location Not Fix':
            if gsmloc_backup:
                # Use CIPGSMLOC for coordinates
                logging.debug(gps_stat + '. Use GSM provided loc.')
                try:
                    # Establish gprs connection if not
                    if not self.module.gprs_is_connected():
                        self.module.gprs_connect()
                    loc = self.module.gsm_loc()
                except Exception as e:
                    logging.error(msg=e.__str__())
                    return False
            else:
                return False
        else:
            # Use more accurate GPS engine
            loc = self.module.gps_data()

        return loc
    
    def track(self):
        # Send location to the user through SMS/BT
        msg = str()
        logging.debug(msg='Tracking vehicle location ... ')
        loc = self.get_location()

        if not loc:
            _msg = 'No GPS data source available!'
            logging.error(msg=_msg)
            msg += 'ERROR: {}\n'.format(_msg)
            return

        else:
            provider = loc.provider
            if loc.is_valid:
                logging.debug(
                    msg='lat: {}; \t lng:{}'.format(loc.lat, loc.lng)
                )
                msg += 'https://maps.google.com/?q={},{}\n'.format(loc.lat, loc.lng)
                msg += 'when: {},{}\n'.format(loc.time, loc.date)
            else:
                _msg = 'No valid GPS data available'
                logging.error(msg=_msg)
                msg += _msg + '\n'

            msg += 'provider: ' + provider + '\n'
            if provider == 'NETWORK':
                msg += 'lcode: ' + loc.lcode + '\n'
        
        # append other data here:
        logging.info('Sending data: {}'.format(msg))
        delay(4) # self.module.send_sms(number, msg)
        logging.info('Sent!')
        
    def ms_enable(self, settings):
        ''' Launch OrientationMonitor thread to sense movement '''
        # settings = dictionary provided by ms_en on main.py

        logging.info('Starting movement_sense thread ...')
        self.sensor.INT_enabled = True  # Setup MPU6050 gyroscope
        self.ms_thread = OrientationMonitor(
            target  = None,
            name    = 'movement_sense',
            kwargs  = {
                'sensor': self.sensor, 
                **settings  # Unpack settings content and
            }
        )
        self.ms_thread.start()
        self.movement_sense = True
        logging.info('movement_sense thread successfully started!')

    def ms_disable(self, *args, **kwargs):
        # Disable movement sensing
        if not self.movement_sense:
            logging.info('movement_sense is not currently running.')
            return
        
        logging.info('Stopping movement_sense thread')
        self.ms_thread.stop()
        self.ms_thread.join()
        self.sensor.INT_enabled = False
        self.movement_sense = False
        logging.info('movement_sense thread stopped!')

    def iot_enable(self, settings):
        ''' Launch OrientationMonitor thread to sense movement '''
        # settings = dictionary provided by iot_en on main.py

        logging.info('Starting iot_feed thread ...')
        self.iot_thread = IoTFeed(
            target  = None,
            name    = 'iot_feed',
            kwargs  = {**settings}  # Unpack settings content
        )
        self.iot_thread.start()
        self.iot_feed = True
        logging.info('iot_feed thread successfully started!')

    def iot_disable(self, *args, **kwargs):
        # Disable iot_feed
        if not self.iot_feed:
            logging.info('iot_feed is not currently running.')
            return
        
        logging.info('Stopping iot_feed thread')
        self.iot_thread.stop()
        self.iot_thread.join()
        self.iot_feed = False
        logging.info('iot_feed thread stopped!')

    # @property
    # def iot_feed(self):
    #     return self._iot_feed

    # @iot_feed.setter
    # def iot_feed(self, value):
    #     # pdb.set_trace()
    #     value = bool(value)

    #     # get iot_feed value
    #     try:
    #         is_running = self.iot_feed
    #     except AttributeError as e:
    #         logging.error(msg=e.__str__() + '. First run.')
    #         is_running = False

    #     if value:
    #         # Enable movement sensing
    #         if is_running:
    #             logging.info('iot_feed is currently running.')
    #             return

    #         # Setup module and run the iot_feed thread
    #         logging.info('Starting iot_feed thread ...')
    #         self.iot_thread = IoTFeed(
    #             target  = None,
    #             name    = 'iot_feed',
    #             kwargs  = {
    #                 'module'        : self.module,
    #                 'tcp_server'    : self.iot_server,
    #                 'sensor'        : self.sensor,
    #                 'get_loc_func'  : self.get_location
    #             }
    #         )
    #         self.iot_thread.start()
    #         self._iot_feed = True
    #         logging.info('iot_feed thread successfully started!')
        
    #     else:
    #         # Disable iot_feed
    #         if not is_running:
    #             logging.info('iot_feed is not currently running.')
    #             return
            
    #         logging.info('Stopping iot_feed thread')
    #         self.iot_thread.stop()
    #         self.iot_thread.join()
    #         self._iot_feed = False
    #         logging.info('iot_feed thread stopped!')
            

class OrientationMonitor(threading.Thread):
    def __init__(self, target, name, args=(), kwargs={}):
        super(OrientationMonitor, self).__init__(
            target=target,
            name=name,
            args=args,
            kwargs=kwargs
        )
        self._stop_event = threading.Event()

    def start(self):
        logging.debug('Starting thread - {}'.format(self.name))
        super().start()

    def stop(self):
        logging.debug('Stopping thread - {}'.format(self.name))
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def _gyro_average(self, samples):
        logging.debug('Getting gyro average reading ...')
        gyro_x_samples = list()
        gyro_y_samples = list()
        gyro_z_samples = list()

        for i in range(samples):
            while not self.sensor.data_ready:
                pass

            gyro_x_samples.append(self.sensor.gyro_read(axis='X'))
            gyro_y_samples.append(self.sensor.gyro_read(axis='Y'))
            gyro_z_samples.append(self.sensor.gyro_read(axis='Z'))
            self.sensor.data_ready = False
            self.sensor.INT_clear()

        gyro_x_ave = sum(gyro_x_samples)/samples
        gyro_y_ave = sum(gyro_y_samples)/samples
        gyro_z_ave = sum(gyro_z_samples)/samples
        logging.debug('Gyro Average -> X: {} \t Y: {} \t Z: {}'\
            .format(gyro_x_ave, gyro_y_ave, gyro_z_ave))
        return gyro_x_ave, gyro_y_ave, gyro_z_ave
            
    def run(self):
        # Get settings
        self.sensor             = self._kwargs['sensor']
        # actions to do when when_moved fires
        sms                     = self._kwargs.get('sms')   # Send SMS
        iot                     = self._kwargs.get('iot')   # Enable iot
        alarm                   = self._kwargs.get('alarm')
        # movement sensing settings
        threshold               = self._kwargs.get('th')
        samples                 = self._kwargs.get('samples') # Averaging
        when_moved              = self._kwargs['callback']
        when_moved_rate         = self._kwargs.get('when_moved_rate')
        # iot settings
        check_loc_delta         = self._kwargs.get('check_loc_delta')
        loc_delta_th            = self._kwargs.get('loc_delta_th')
        
        gyro_current = self._gyro_average(samples=samples)
        
        logging.debug(msg='Entering Movement monitoring ...')
        while not self.stopped():
            
            if not self.sensor.data_ready:
                continue

            gyro_new = self.sensor.gyro_read()
            for i in range(3):
                if abs(gyro_new[i]-gyro_current[i]) > threshold:
                    logging.debug(msg='----- Sensed movement! -----')

                    # Call when_when moved function
                    when_moved(
                        sms=sms,
                        iot=iot,
                        alarm=alarm,
                        check_loc_delta=check_loc_delta,
                        loc_delta_th=loc_delta_th
                    )
                    # TODO: Add exception handling 
                    delay(when_moved_rate)

                    logging.debug(msg='---- Getting new samples ---')
                    gyro_new = self._gyro_average(samples=samples)
                    break # Exit from for loop
            
            self.sensor.data_ready = False
            self.sensor.INT_clear()

        logging.debug(msg='Exiting from Movement monitoring ...')
        self._stop_event.clear()


class IoTFeed(threading.Thread):

    def __init__(self, target, name, args=(), kwargs={}):
        super(IoTFeed, self).__init__(
            target=target,
            name=name,
            args=args,
            kwargs=kwargs
        )
        self._stop_event = threading.Event()
        self._tcp_commanderr_handler_counter = 0
        self._tcp_timeouterr_handler_counter = 0

    def start(self):
        logging.debug('Starting thread - {}'.format(self.name))
        super().start()

    def stop(self):
        logging.debug('Stopping thread - {}'.format(self.name))
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    # def _tcp_timeouterr_handler(self):
    #     logging.info('Running TimeoutExpired handler ...')

    #     if not self.module.gprs_is_connected():
    #         self.module.gprs_connect()
        
    #     # Reset TCP connection
    #     try:
    #         self.module.tcp_close()
    #     except Exception as e:
    #         logging.error(msg=e.__str__())

    #     try:
    #         self.module.tcp_connect(self.tcp_server)
    #     except Exception as e:
    #         logging.error(msg=e.__str__())
        
        # try:
        #     self.module.tcp_close()
        # except CommandError as e:
        #     logging.debug(msg=e.__str__())
        # try:
        #     self.module.tcp_connect(self.tcp_server)
        # except TimeoutExpired as e:
        #     self._tcp_timeouterr_handler()
        # except CommandError as e:
        #     self._tcp_timeouterr_handler()


    # def _tcp_commanderr_handler(self):
    #     logging.debug('Running _tcp_commanderr_handler ...')

    #     try:
    #         self.module.tcp_close()
    #     except CommandError as e:
    #         logging.error(msg=e.__str__())
        
    #     try:
    #         self.module.tcp_connect(self.tcp_server)
    #     except TimeoutExpired as e:
    #         logging.error(msg=e.__str__())
    #         self._tcp_timeouterr_handler()
    #     except CommandError as e:
    #         logging.error(msg=e.__str__())
    #         self._tcp_commanderr_handler()


    # def tcp_packet(self, temp=True, *args, **kwargs):
    #     ''' format data to send on tcp_server '''
    #     # See ubidots docs: https://ubidots.com/docs/hw/#tcp--udp
    #     context = ''
    #     body = '''tracknroll|POST|A1E-YAmgm46yp3XrCtoYWou3dd8pcwfGYU|tracknroll=>temp:{}{}@{}|end'''
    #     temp = self.sensor.temp_read()
    #     # pdb.set_trace()
    #     gps_stat = self.module.gps_stat()

    #     context += '$rssi=' + self.module.signal_strength()
    #     context += '$gps_stat=' + gps_stat

    #     # _filter lambda func
    #     _filter = lambda key: key in ('utc_time', 'stat', 'course', 'date', 'time')
    #     gps_data = self.get_loc(gsmloc_backup=kwargs.get('gsmloc_backup')) # this is a GPSData object
    #     if gps_data:
    #         for k,v in gps_data.__dict__.items():
    #             # filter data to reduce tcp_packet size ('utc_time', 'stat')
    #             if _filter(k):
    #                 continue    # do not add on context var
    #             context += '${}={}'.format(k, v)

    #     return body.format(temp, context, int(now())*1000)

    def run(self):
        # Get settings
        update_rate         = self._kwargs.get('update_rate')
        update_ubidots      = self._kwargs.get('callback')
        
        while not self.stopped():
            update_ubidots(
                ua              = self._kwargs.get('ua'),
                token           = self._kwargs.get('token'),
                dev_label       = self._kwargs.get('dev_label'),
                dev_name        = self._kwargs.get('dev_name'),
                var_name        = self._kwargs.get('var_name'),
                ubi_tcp_body    = self._kwargs.get('ubi_tcp_body'),
                gsmloc_backup   = self._kwargs.get('gsmloc_backup')
            )

            delay(update_rate)

    # def run(self):
        
    #     self.module     = self._kwargs['module']
    #     self.tcp_server = self._kwargs['tcp_server']
    #     self.sensor     = self._kwargs['sensor']
    #     self.get_loc    = self._kwargs['get_loc_func']

    #     # gps_pwr_stat = self.module.gps_pwr          # Store prev stat of gps, restore before exit
    #     self.module.gps_pwr = 1                     # Turn ON gps
    #     # temp_sense_stat = self.sensor.temp_enabled 
    #     self.sensor.temp_enabled = True

    #     # Connect to GPRS network
    #     if not self.module.gprs_is_connected():
    #         self.module.gprs_connect()
        
    #     # try:
    #     #     self.module.tcp_connect(self.tcp_server)
    #     # except CommandError as e:
    #     #     logging.error(msg=e.__str__())

    #     self.module.call_method(
    #         cmd=self.module.tcp_connect,
    #         server=self.tcp_server,
    #         pre_reset=self.module.gprs_close,
    #         post_reset=self.module.gprs_connect,
    #         e_handler={
    #             'TimeoutExpired'    : self._tcp_timeouterr_handler,
    #             'CommandError'      : self._tcp_timeouterr_handler,
    #         }
    #     )

    #     # Feed IoT server with data
    #     while not self.stopped():

    #         tcp_packet = self.tcp_packet(gsmloc_backup=False)

    #         self.module.call_method(
    #             cmd=self.module.tcp_send,
    #             packet=tcp_packet,
    #             pre_reset=self.module.gprs_close,
    #             post_reset=self.module.gprs_connect,
    #             e_handler={
    #                 'TimeoutExpired'    : self._tcp_timeouterr_handler,
    #                 'CommandError'      : self._tcp_timeouterr_handler,
    #             }

    #         )

    #         delay(5)

            # try:
            #     self.module.tcp_send(self.tcp_packet())
            # except TimeoutExpired as e:   # Need to reset conn
            #     logging.error(msg=e.__str__())
            #     self._tcp_timeouterr_handler()
            #     continue
            # except CommandError as e:   # TCP conn closed
            #     logging.error(msg=e.__str__())
            #     self._tcp_commanderr_handler()
            #     continue
            # except Exception as e:
            #     logging.error(msg=e.__str__())

            # delay(5)

        # self.sensor.temp_enabled = temp_sense_stat
        # self.module.gps_pwr = gps_pwr_stat




# """ ===================================================================== TEST CODE ======================================================================= """

# s = MPU6050(bus=1, INT_pin=4)

# gyro_thread = None

# def client_connected():
#     logging.debug(msg='BT client connected!')
#     global gyro_thread
#     if not gyro_thread:
#         return
#     gyro_thread.stop()
#     gyro_thread.join()
#     logging.debug(msg='Movement sensing disabled ... ')
#     s.INT_enabled = False

# def client_disconnected():
#     logging.debug(msg='BT client disconnected!')
#     s.INT_enabled = True
#     global gyro_thread
#     gyro_thread = OrientationMonitor(
#         target=None,
#         name='OrientationMonitor',
#         kwargs={
#             'sensor':   s,
#             'gyro_fsr': 1,
#             'th':       50,
#             'samples':  10
#         }
#     )
#     logging.debug(msg='Starting Movement detection thread!')
#     gyro_thread.start()


# if __name__ == '__main__':
#     btserver = BluetoothServer(
#         data_received_callback=None,
#         power_up_device=True,
#         when_client_connects=client_connected,
#         when_client_disconnects=client_disconnected,
#     )

#     if not btserver.client_connected:
#         client_disconnected()

#     while True:

#         while not s.data_ready:
#             continue

#         temp = s.temp_read()
#         logging.info('Temperature: {}'.format(temp))
#         delay(1)

#         s.data_ready = False
#         s.INT_clear()


#     pause()

