import logging, threading, os, json, pdb
from sys import stdout as sys_stdout
from core.sensor import MPU6050
from core.general import Vehicle, Command, ThreadMonitor
from core.module import SIM808, GPRSProfile, TCPServer
from core.module import TimeoutExpired, CommandError
from gpiozero import Button, LED
# from bluedot.btcom import BluetoothServer
from time import sleep as delay
from time import time as now
from pytz import utc, timezone
from signal import pause

# ==========================================================================================================
# Setup Logging
# ==========================================================================================================
base_dir = os.path.dirname(os.path.abspath(__file__))
log_file = os.path.join(base_dir, 'tracknroll.log')

# Create log file if not exists
if 'tracknroll.log' not in base_dir:
    with open(log_file, 'w') as f:
        pass

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler(sys_stdout)
    ]
)

# ==========================================================================================================
# Load/Update global settings (settings.json). Vars should be updated everytime user changes settings.
# ==========================================================================================================
settings = dict()   # global setting stored here
settings_file = os.path.join(base_dir, 'settings.json') # settings.json path

def load_settings():
    # Read settings.json file and load value to settings dict
    global settings, settings_file
    with open(settings_file, 'r') as f:
        settings = json.load(f)

def _num_from_settings():
    from base64 import b64decode
    return b64decode(settings['security'].get('num')).decode()

load_settings() # Load global settings

# ==========================================================================================================
# Date, Time, and Timezone management
# ==========================================================================================================
SYSTEM_TIME_UPDATED = False
tz = timezone(settings['system'].get('timezone'))

def update_system_time(source):
    '''
        update system time with accurate source time.
        source -> utc localized datetime.datetime object (from GPS)
    '''
    global tz
    dt_local = source.astimezone(tz).strftime('%c') # localize time
    logging.info('Updating system time: {}'.format(dt_local))
    os.system('date --set="{}"'.format(dt_local)) # use date cmd to update system time

# ==========================================================================================================
# Instantiation
# ==========================================================================================================
module = SIM808(port='/dev/ttyS0', gprs_inactive_timo=5, power_monitor=True, pkey_pin=LED(27), stat_pin=Button(18, pull_up=True))
gprs_profile = GPRSProfile(cid=1, apn=settings['module'].get('apn', "internet.globe.com.ph"))
module.gprs_profile = gprs_profile
ubidots = TCPServer('things.ubidots.com', 9012)
sensor = MPU6050(bus=1, INT_enabled=True, INT_pin=Button(4))
motor = Vehicle(module=module, iot_server=ubidots, sensor=sensor)
# thread_monitor = ThreadMonitor()

# ==========================================================================================================
# Movement Sensing Implementation
# ==========================================================================================================
def msg_when_moved(*args, **kwargs):
    logging.info('Sending msg_when_move sms ...')
    # module.send_sms(
    #     number=settings.get('number'),
    #     message='movement sensed'
    # )
    delay(10)
    logging.info('SMS sent!')

def get_distance(lat1, lng1, lat2, lng2):
    '''
    Calculate distance between two coordinates using Haversine formula
    lat1, lng1 -> latitude and longitude of coordinate 1
    lat2, lng2 -> latitude and longitude of coordinate 2
    returns distance in meters
    '''
    from math import radians, cos, sin, asin, sqrt

    # convert decimal degrees to radians 
    lat1, lng1, lat2, lng2 = map(radians, [lat1, lng1, lat2, lng2])

    # haversine formula 
    dlng = lng2 - lng1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlng/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6378 # Radius of earth in kilometers. Use 3956 for miles
    return (c * r) * 1000   

def when_moved(sms, iot, alarm, check_loc_delta, loc_delta_th, *args, **kwargs):
    logging.info('Executing when_moved function ...')

    logging.debug(
        msg='sms: {} \t iot: {} \t loc_delta_th: {}'.format(
            sms, iot, loc_delta_th
            )
    )

    if alarm:
        logging.debug('Alarm activated!')
        delay(5)
        pass

    if (sms | iot) & check_loc_delta:
        # Get GPS fix status
        gps_stat = module.gps_stat()
        logging.info('gps_stat: {}'.format(gps_stat))

        if gps_stat == 'Location 3D Fix':
            # park_loc: initial vehicle location. to be used in loc_delta_th
            # calculation. Only send iot update if the vehicle current location
            # is greater than loc_delata_th away from park_loc
            if not motor.park_loc:
                # get park_loc first
                logging.info('Need to get park_loc first!')
                motor.park_loc = motor.get_location(gsmloc_backup=False)
                logging.info('park_loc: {}'.format(str(motor.park_loc)))
                return

            # Get current_loc and calculate distance from park_loc
            current_loc = motor.get_location(gsmloc_backup=False)
            loc_delta = get_distance(
                lat1=current_loc.lat,
                lng1=current_loc.lng,
                lat2=motor.park_loc.lat,
                lng2=motor.park_loc.lng
            )

            logging.info('Location change: {} meters.'.format(loc_delta))

            # if loc_delat > loc_delata_th, then the vehicle was moved
            if loc_delta >= loc_delta_th:
                logging.info(
                    msg='loc_delta: {} reached loc_delta_th. Vehicle loc changed!'.format(loc_delta)
                )
                motor.park_loc = current_loc
            else:
                return

        else:
            logging.error('GPS no sattelite fix. Unable to calculate loc_delta! ...')
            logging.debug('when_moved routine done!')
            return
    
    if sms:
        logging.debug('Running send_sms funtion!')
        delay(5)
    if iot:
        if motor.iot_feed:
            logging.info('iot_feed is running. No need to run update_ubidots ...')
        else:
            logging.debug('Running iot_update function!')
            readbk = update_ubidots(**settings.get('iot'))
            if 'CLOSE' in readbk:
                update_ubidots(**settings.get('iot'))
    
    logging.debug('when_moved routine done!')

def ms_en(kwargs):
    '''Launch vehicle movement_sense thread'''
    logging.info('Setting movement sensing ...')
    ms_settings = settings['ms'] # Get movement_sense global settings

    if motor.movement_sense:
        logging.info('movement_sense running. resetting movement_sense!')
        ms_dis()

    new_settings = dict()   # Settings to be applied
    for key in ms_settings.keys(): # Override global settings
        new_settings[key] = kwargs.get(key, ms_settings[key])

    # Add callback function on settings
    new_settings['callback'] = when_moved

    logging.info('Movement Sensing settings: {}'.format(new_settings.__str__()))
    
    # Initialize needed resources
    if new_settings.get('check_loc_delta'):  # GPS
        # Power up GPS
        if module.gps_pwr != 1:
            logging.debug('Powering GPS ...')
            module.gps_pwr = 1
            delay(10)
        # GPS fix needed
        if module.gps_stat() == 'Location 3D Fix':
            motor.park_loc = motor.get_location(gsmloc_backup=False)
        else:
            motor.park_loc = None
        logging.info('Park location: {}'.format(str(motor.park_loc)))
    
    if new_settings.get('iot'):  # GPRS
        if not module.gprs_is_connected():
            module.gprs_connect()

    motor.ms_enable(settings=new_settings)

def ms_dis(*args, **kwargs):
    # motor.park_loc = None
    if not motor.movement_sense:
        logging.debug('Movement sense already disabled!')
        return
    motor.ms_disable()

# ==========================================================================================================
# IoT server Update (ubidots.com/education)
# ==========================================================================================================
def update_ubidots(ua, token, dev_label, dev_name, var_name, ubi_tcp_body, gsmloc_backup, *args, **kwargs):
    ''' Note: use this function only once GPS establish satellite fix '''
    context = ''
    temp = sensor.temp_read()
    gps_stat = module.gps_stat()

    context += '$rssi=' + module.signal_strength()
    context += '$gps_stat=' + gps_stat

    gps_data = motor.get_location(gsmloc_backup=False)
    
    # RPi system time update
    global SYSTEM_TIME_UPDATED
    if not SYSTEM_TIME_UPDATED:
        update_system_time(source=gps_data._when)
        SYSTEM_TIME_UPDATED = True

     # _filter lambda func
    _filter = lambda key: key in ('utc_time', 'stat', 'course', 'date', 'time', 'when', 'timestamp', 'lcode', '_when')
    if gps_data:
        for k,v in gps_data.__dict__.items():
            # filter data to reduce tcp_packet size ('utc_time', 'stat')
            if _filter(k):
                continue    # do not add on context var
            context += '${}={}'.format(k, v)

    timestamp_ms = gps_data.timestamp() * 1000  # timestamp in milliseconds
    
    # UA TOKEN DEV_LABEL VAR_NAME VAR_VAL CONTEXT TIMESTAMP
    tcp_packet = ubi_tcp_body.format(ua, token, dev_label, var_name, temp, context, timestamp_ms)

    # Sent tcp_packet
    readbk = module.call_method(
        cmd=module.tcp_send,
        packet=tcp_packet,
        pre_reset=module.gprs_close,
        post_reset=module.gprs_connect,
        e_handler={
            'TimeoutExpired'    : te_handler,
            'CommandError'      : te_handler,
        }
    )

    return readbk

def te_handler():
    logging.info('Running TimeoutExpired handler ...')

    if not module.gprs_is_connected():
        module.gprs_connect()
    
    # Reset TCP connection
    try:
        module.tcp_close()
    except Exception as e:
        logging.error(msg=e.__str__())

    try:
        module.tcp_connect(ubidots)
    except Exception as e:
        logging.error(msg=e.__str__())

def ce_handler():
    pass

def iot_en(kwargs):
    # kwargs: sms provided settings
    logging.debug('Setting iot_feed settings ...')
    iot_settings = settings.get('iot')

    if motor.iot_feed:
        logging.info('iot_feed running. resetting iot_feed.')
        iot_dis()
    
    new_settings = dict()   # Settings to be applied
    for key in iot_settings.keys(): # Override global settings
        new_settings[key] = kwargs.get(key, iot_settings[key])
    new_settings['callback'] = update_ubidots
    logging.info('iot_feed settings: {}'.format(new_settings.__str__()))

    # Connect module to GPRS network if not connected
    if not module.gprs_is_connected():
        module.gprs_connect()

    # Power on GPS if not on
    if not module.gps_pwr:
        module.gps_pwr = 1

    motor.iot_enable(settings=new_settings)

def iot_dis(*args, **kwargs):
    motor.iot_disable(*args, **kwargs)

# ==========================================================================================================
# Util Commands
# ==========================================================================================================
def get_stats(kwargs):
    # Get device status
    # temp, vbt_stat, gps_stat, 
    temp = sensor.temp_read()
    vbt = module.vbt_stat()
    vbt_charging = vbt.is_charging
    vbt_percent = vbt.percent
    vbt_voltage = vbt.voltage
    gps_pwr = module.gps_pwr
    gps_stat = module.gps_stat()
    gprs_stat = module.gprs_is_connected()

    stat = '--- TrackNRoll Stats ---\n'
    stat += 'temp: {}\n'.format(temp)
    stat += 'vbt_charging: {}\n'.format(vbt_charging)
    stat += 'vbt_percent: {}\n'.format(vbt_percent)
    stat += 'vbt_voltage: {}\n'.format(vbt_voltage)
    stat += 'gps_power: {}\n'.format(gps_pwr)
    stat += 'gps_stat: {}\n'.format(gps_stat)
    stat += 'network_reg: {}\n'.format(module.network_reg_stat())
    stat += 'signal_strength: {}\n'.format(module.signal_strength())
    stat += 'gprs_connected: {}'.format(gprs_stat)

    if kwargs.get('rep_here'):
        number = kwargs.get('sender')
    else:
        number = _num_from_settings()
    number = '+63' + number

    print(stat)
    try:
        module.send_sms(number=number, message=stat)
    except Exception as e:
        logging.error(msg=e.__str__())

def get_settings(kwargs):
    s = str()
    with open(settings_file, 'r') as f:
        for _ in f.readlines(): s += _
    
    if kwargs.get('rep_here'):
        number = kwargs.get('sender')
    else:
        number = _num_from_settings()
    number = '+63' + number

    logging.debug('Sending settings to {}'.format(number))
    # module.send_sms(number=number, message=s)

def update_settings(kwargs):
    # kwargs: dict. key-value pair of settings to update
    global settings, settings_file

    if kwargs.get('pw'):
        from bcrypt import hashpw, gensalt
        hashed_pw = hashpw(
            kwargs.get('pw').encode(), 
            gensalt()
        )
        kwargs['pw'] = hashed_pw.decode()

    if kwargs.get('num'):
        from core.general import to10digit_num
        from base64 import b64encode
        num = to10digit_num(kwargs.get('num'))
        b64_num = b64encode(num.encode())
        kwargs['num'] = b64_num.decode()

    for settings_subsets in settings.keys():
        for k,v in kwargs.items():
            if k in settings[settings_subsets].keys():
                settings[settings_subsets].update({k: v})

    with open(settings_file, 'w') as f:
        json.dump(settings, f, indent=4)

# ==========================================================================================================
# Top Levels
# ==========================================================================================================
Command.cmd_map = {
    'tr'    : motor.track,
    'gst'   : get_stats,
    'us'    : update_settings,
    'gs'    : get_settings,
    'me'    : ms_en,
    'md'    : ms_dis,
    'ie'    : iot_en,
    'id'    : iot_dis
}

def sms_command_manager(debug=False, sms=None):
    # Get REC_UNREAD Messages
    logging.debug(msg='sms_command_manager thread running ...')

    if debug and sms:
        unread_messages = (sms,)
    else:
    # Need to run list_sms twice. Sometimes returns nothing
    # on first call even if an unread message exists.
        try:
            unread_messages = tuple(module.list_sms('REC UNREAD'))
        except TimeoutExpired as e:
            logging.error(msg=e.__str__())
            return

        if not unread_messages:
            try:
                unread_messages = tuple(module.list_sms('REC UNREAD'))
            except TimeoutExpired as e:
                logging.error(msg=e.__str__())
                return
            else:
                if not unread_messages:
                    logging.debug('RI False Triggered!')
                    return

    # Loop on Unread messages and check for command
    for sms in unread_messages:
        logging.debug(msg='Reading new message ... ')
        logging.debug(msg=' '.join(sms))
        command = Command(sms, pw_hashed=settings['security'].get('pw', ''))
        if command.is_valid:
            # Execute command
            cmd_thread = threading.Thread(
                target=command.execute,
                name=command.command,
            )
            cmd_thread.daemon = True
            cmd_thread.start()

        else:
            logging.error(
                msg='Wrong password/command: {}'.format(' '.join(sms))
            )

        module.delete_sms(typ='READ')

def new_sms_received():
    # launch sms_command_manager thread
    logging.info('RI pin interrupt!')
    cmd_mngr = threading.Thread(
        target=sms_command_manager,
        name='sms_command_manager'
    )
    cmd_mngr.daemon = True
    logging.debug('Running sms_command_manager thread ...')
    cmd_mngr.start()

def new_btdata_received(data):
    logging.debug('Bluetooth data received: {}'.format(data))

    cmd_thread = threading.Thread(
        target=Command.cmd_map[data],
        name=data
    )

    logging.debug('Executing BT command: {}'.format(data))
    cmd_thread.daemon = True
    cmd_thread.start()

if __name__ == '__main__':
    # bt_server = BluetoothServer(
    #     data_received_callback=new_btdata_received,
    # )

    RI = Button(17, bounce_time=50e-3)
    RI.when_released = new_sms_received

    # Check for unread messages
    sms_command_manager()

    sms_me_warg = ('1', 'REC UNREAD', '09051577637', 'now', 'now', '1234 me check_loc_delta=True iot=True alarm=False')
    sms_command_manager(debug=True, sms=sms_me_warg)

    pause()