import logging, pdb
from time import sleep as delay
from time import time as now
from pytz import utc
from datetime import datetime
from threading import Thread, RLock, Event
from serial import Serial
from collections import namedtuple
from re import compile as regex_compile
from sys import stdout as sys_stdout

MODULE_INFO = namedtuple('MODULE_INFO', 
    ('man','model', 'rev', 'imei', 'network', 'mobile_num')
)

VBAT_STAT = namedtuple('VBAT_STAT', ['is_charging', 'percent', 'voltage'])

# logging.basicConfig(
#     level=logging.DEBUG,
#     format='%(asctime)s - %(levelname)s - %(message)s',
#     handlers=[
#         # logging.FileHandler('test.log'),
#         logging.StreamHandler(sys_stdout)
#     ]
# )

''' Helper Funcs '''
has_err = lambda s: s.__contains__('ERR')

def err_check(readbk, cmd, optional=''):
    if not readbk:
        raise TimeoutExpired(cmd=cmd)

    if tuple(filter(has_err, readbk)):
        raise CommandError(
            error=readbk[0],
            cmd=cmd,
            optional=optional
        )

""" -------------------- Class Definitions -------------------- """
class InactiveMonitor(Thread):

    def __init__(self, target, name, args=(), kwargs={}):
        super(InactiveMonitor, self).__init__(
            target=target,
            name=name,
            args=args,
            kwargs=kwargs
        )
        self._stop_event = Event()

    def start(self):
        logging.debug('Starting thread - {}'.format(self.name))
        super().start()

    def stop(self):
        logging.debug('Stopping thread - {}'.format(self.name))
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

    def run(self):
        timeout = self._kwargs['timeout'] * 60
        module = self._kwargs['module']

        while not self.stopped():
            tdelta = now() - module._gprs_last_used
            if tdelta > timeout:
                logging.debug('GPRS connection has been inactive for {} minutes. Closing conn to conserve power.'.format(tdelta/60))
                with module.uart_lock:
                    module.gprs_close()
            # logging.debug(msg=str(now() - module._gprs_last_used))
            delay(1)

        logging.debug(msg='Exiting from InactiveMonitor thread')
        self._stop_event.clear()

class SimModule(Serial):
    
    def __init__(self, port, timeout=5, baudrate=9600, power_monitor=False, gprs_inactive_timo=None, *args, **kwargs):
        super().__init__(port=port, timeout=timeout, baudrate=baudrate)
        self.uart_lock = RLock()
        self._gprs_last_used_lock = RLock()
        self.gprs_inactive_timo = gprs_inactive_timo

        if power_monitor and kwargs.get('pkey_pin') and kwargs.get('stat_pin') :
            self._pkey_pin = kwargs['pkey_pin']
            self._stat_pin = kwargs['stat_pin']
            # Power UP module if not powered up
            self.power = True
            self.power_monitor = True
            # logging.debug('power_monitor: {}'.format(self.power_monitor))
            # logging.debug('_when_released: {}'.format(self._stat_pin.when_released.__name__))
        else:
            self.power_monitor = False

        self.initialize()

    def __str__(self):
        return ' '.join(self.module_info)

    @staticmethod
    def dm_to_dd(dm):   # coordinate conversion to decimal digits
        regex_dd = regex_compile(r'(.*)(\d\d\..*)')
        dd = regex_dd.search(dm)
        dd = int(dd.group(1)) + (float(dd.group(2))/60)
        return round(dd, 6)

    @staticmethod
    def utc_to_utcplus8(utc):
        # utc: HHMMSS.SSS
        # choose manual formating to avoid using datetime
        hour = int(utc[0:2]) + 8    # utc+8
        return str(hour) + ':' + utc[2:4] + ':' + utc[4:]

    @staticmethod
    def format_date(ddmmyy):
        # choose manual formating to avoid using datetime
        # returns dd-mm-yy
        return ddmmyy[0:2] + '-' + ddmmyy[2:4] + '-' + ddmmyy[4:6]

    @staticmethod
    def rssi(rssi):
        # Utility function for rssi dBm conversion
        if rssi == 0:
            _rssi = -115
        elif rssi == 1:
            _rssi = -111
        elif rssi >= 2 and rssi <= 30:
            # formula from Arduino map function
            # (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            _rssi = (rssi - 2) * (-54 + 110) / (30 - 2) - 110
        elif rssi == 31:
            _rssi = -52
        elif rssi == 99:
            return 'UNKNOWN'
            
        return str(_rssi) + ' dBm'

    def close(self):
        try:
            is_conn = self.gprs_is_connected()
        except AttributeError:
            pass
        else:
            if is_conn:
                self.gprs_close()
        finally:
            super().close()

    def vbt_stat(self):
        logging.debug('Reading VBAT status ...')
        cmd = 'at+cbc'
        readbk = self.write_cmd(cmd=cmd)
        err_check(readbk, cmd)
        vbat = readbk[0].replace('+CBC: ', '')
        vbat = vbat.split(',')
        vbat[0] = bool(int(vbat[0]))
        vbat[1] = int(vbat[1])
        vbat[2] = int(vbat[2])/1000
        rtn =  VBAT_STAT(*vbat)
        logging.debug(msg=rtn.__str__())
        return rtn

    @property
    def vbt_charge(self):
        return self._vbt_charge
    
    @vbt_charge.setter
    def vbt_charge(self, val):
        if val != 0:
            self._vbt_charge = 1
            logging.info('Enable battery charging ...')
        else:
            self._vbt_charge = 0
            logging.info('Disable battery charging ...')
        
        cmd = 'at+echarge={}'.format(self._vbt_charge)
        readbk = self.write_cmd(cmd=cmd)
        err_check(readbk, cmd)

    def _toggle_pkey(self):
        self._pkey_pin.on()
        delay(1.5)
        self._pkey_pin.off()

    @property
    def power(self):
        return self._stat_pin.is_active

    @power.setter
    def power(self, value):
        # pdb.set_trace()
        if value:
            if self.power:
                return
            self._toggle_pkey()
        
        if not self.power:
            return
        self._toggle_pkey()
        
    def power_reset(self):
        logging.debug('Performing power reset ...')
        if not self.power:  # if powered down
            self.power = 1  # power up
        else:   # if powered up,
            self._toggle_pkey() # power down
            delay(10)

    @property
    def power_monitor(self):
        return self._power_monitor
    
    @power_monitor.setter
    def power_monitor(self, value):

        if not value:
            self._stat_pin.close()
            self._pkey_pin.close()
            self._power_monitor = False
        else:
            # for attr in []
            self._stat_pin.when_released = self.initialize
            self._power_monitor = True

    def _power_interrupted_handler(self):
        logging.critical(msg='Power interrupted! Powering module ...')
        self.initialize()


    def call_method(self, cmd, max_try=10, dly=2, e_handler={}, pre_reset=None, post_reset=None, *args, **kwargs):
        try_count = 0

        while True:
            if try_count >= max_try:
                logging.critical(
                    msg='Max tries reached! Performing module power reset.'
                )
                if pre_reset:
                    logging.info('Running pre-reset function ...')
                    try:
                        pre_reset()
                    except Exception as e:
                        logging.error('pre_reset exception: {}'.format(e.__str__()))
                self.power_reset()
                if post_reset:
                    logging.info('Running post-reset function ...')
                    try:
                        post_reset()
                    except Exception as e:
                        logging.error('post_reset exception: {}'.format(e.__str__()))
                try_count = 0
            
            try:
                readbk = cmd(*args, **kwargs)
            except Exception as e:
                # pdb.set_trace()
                logging.error(msg=e.__str__())
                e_name = e.__class__.__name__       # Get exception name
                handler = e_handler.get(e_name)     # Get exception handler 
                if handler:
                    logging.info('Calling {} exeption handler ...'.format(e_name))
                    handler()                 # Call handler
                try_count += 1
                logging.debug(msg='Try counts: {}'.format(try_count))
                delay(dly)
                continue
            else:
                return readbk
                
    def initialize(self):
        ''' Module initialization routine '''
        # Power up module if powered down
        self.power = 1

        with self.uart_lock:
            # 1. Synchronize
            self.call_method(cmd=self._synchronize, max_try=5)

            # 2. Module settings
            self.write_cmd(cmd='at+cipqsend=0') # Normal send mode. SEND OK
            self.write_cmd(cmd='at+echarge=1')  # Enable VBAT charging

            # 3. Wait for network connection
            logging.debug(msg='Waiting for network connection ...')
            while True:
                if self.call_method(cmd=self.network_reg_stat):
                    break
                delay(5)

            # 3. Module information
            self._module_info()

    def _synchronize(self):
        logging.debug('Synchronizing module ...')
        cmd = 'at'
        readbk = self.write_cmd(
            cmd=cmd,
            read_timo=10
        )
        logging.debug('Sync: {}'.format(' '.join(readbk)))
        err_check(readbk, cmd)
    
    def _module_info(self):
        cmd = 'at+gsv'
        readbk = self.write_cmd(cmd=cmd)
        err_check(readbk, cmd)
        manufacturer = readbk[0]
        model = readbk[1]
        rev = readbk[2].strip('Revision:')

        cmd = 'at+gsn'
        readbk = self.write_cmd(cmd=cmd)
        err_check(readbk, cmd)
        imei = readbk[0]

        cmd = 'at+cops?'
        readbk = self.write_cmd(cmd=cmd)
        err_check(readbk, cmd)
        cops_regex = regex_compile(r'"(.*)"')
        cops = cops_regex.search(readbk[0])
        cops = cops.group(1)

        cmd = 'at+cnum'
        readbk = self.write_cmd(cmd=cmd)
        err_check(readbk, cmd)
        num_regex = regex_compile(r'\d{9}')
        num = num_regex.search(readbk[0])
        num = num.group()

        self.module_info = MODULE_INFO(
            manufacturer, model, rev, imei, cops, num
        )

    def signal_strength(self):
        logging.debug('Getting signal strength ...')
        cmd = 'at+csq'
        readbk = self.write_cmd(cmd=cmd)
        logging.debug('Raw signal strength: {}'.format(' '.join(readbk)))
        err_check(readbk, cmd)
        readbk = readbk[0]
        readbk = readbk.split(',')
        rssi = readbk[0].replace('+CSQ: ', '')
        rssi = int(rssi)

        rssi = SimModule.rssi(rssi)
        logging.debug('Signal strength: {}'.format(rssi))
        return rssi

    def network_reg_stat(self):
        '''
        Network registration status. Returns boolean.
        True    ->  Connected to a network
        False   ->  Not
        '''
        logging.debug('Getting network registration status ...')
        cmd = 'at+creg?'
        readbk = self.write_cmd(cmd=cmd)
        logging.debug('Network reg. stat: {}'.format (' '.join(readbk)))
        err_check(readbk, cmd)
        _network_reg = readbk[0].replace('+CREG: 0,', '')
        if _network_reg == '1':
            return True
        return False

    def write_cmd(self, cmd, expected_response='OK', read_timo=None, crlf=True, *args, **kwargs):
        '''
        Writes an AT command to the module then reads the modules respone
        to the command. Example: self.send_cmd('AT+CPOWD=1')
        * cmd:                  AT Command
        * expected_response:    A keyword that triggers the method to return immediately
                                if the keyword is present on the response.
        * read_timo:            Read timeout. Stop waiting for the expected_response keyword
                                and return immediately after the timeout. If None, it will
                                wait forever untill the expected_response keyword is present
        * crlf:                 if True, append carriage return and line feed to the command

        '''
        if read_timo:
            orig_timo = self.timeout
            self.timeout = read_timo
        if crlf:
            cmd += '\r\n'
        cmd = cmd.encode()  # Convert to bytes.

        with self.uart_lock:
            # First of all, clear the buffers
            self.reset_input_buffer()
            self.reset_output_buffer()
            self.write(cmd)  # Write the cmd on the module

            # Get response from the written command
            readbk = str()  # store read value here
            resp_len = int()
            if expected_response:
                if isinstance(expected_response, str):
                    expected_response = tuple(expected_response.splitlines())
                resp_len = min(map(len, expected_response))

            while True:
                if expected_response and (len(readbk) >= resp_len):
                    if tuple(filter(readbk.__contains__, expected_response)):
                        break
                tmp = self.read()  # Read a byte
                if not tmp:  # if finished reading
                    break
                try:
                    readbk += tmp.decode()
                except UnicodeDecodeError as e:
                    raise ModuleResponseError(
                        error=e.__str__(), resp=tmp, optional=''
                    )
                
        if read_timo:
            self.timeout = orig_timo

        return tuple(filter(None, readbk.splitlines()))

    ''' -------------------- SMS Methods -------------------- '''
    
    def send_sms(self, number, message):
        '''
        Send SMS message to the number provided. Returns 'OK' if send
        successfully. 'SEND FAIL' otherwise.
        number  ->  str. Recepients cell number. e.g 09xxxxxxxxx
        message ->  str. SMS message body.
        '''
        logging.debug(msg='Sending SMS: {} to {}'.format(message, number))
        cmd = 'at+cmgs="{}"'.format(number)

        with self.uart_lock:
            readbk = self.write_cmd(
                cmd='at+cmgs="{}"'.format(number),
                expected_response='>',
            )

            if not readbk:  # Causes thread to block. Reset module to recover
                logging.error('expecting \'>\' but got empty readbk on send_sms. Reset module to recover.')
                self.power_reset()
                self.initialize()
                readbk = () # let the caller handle the exception raised by err_check
            err_check(readbk, cmd)

            readbk = self.write_cmd(
                cmd=message + '\x1A',
                read_timo=60,
                crlf=False
            )

        logging.debug(msg='Sending result: {}'.format(' '.join(readbk)))
        err_check(readbk, cmd)

        return ' '.join(readbk)

    def read_sms(self, index, mode=0):
        '''
        Returns message details and content as tuple. Returns None
        if no stored sms on the index provided.
        index:  integer value where to read sms from the sim memory
        mode:   if 1, keeps the status unchanged. e.g sms stat will
                remain REC UNREAD after reading it. Default value is
                0, the normal mode, it changes read message stat e.g
                from REC UNREAD to REC READ.
        '''
        logging.debug(msg='Reading SMS at index {}'.format(index))
        cmd = 'at+cmgr={},{}'.format(index, mode)
        readbk = self.write_cmd(cmd=cmd)
        err_check(readbk, cmd)

        if len(readbk) < 3:  # readbk = (details, content, OK)
            logging.debug(msg='EMPTY')
            return None
        
        msg = readbk[0].replace('"', '')    # Remove empty ""
        msg = msg[7:].split(',')            # Remove '+CMGI: ' then split
        msg.remove('')                      # Remove empty strings on the details
        msg.append(readbk[1])               # Append sms body
        msg.insert(0, str(index))
        logging.debug(msg=' '.join(msg))
        return tuple(msg)

    def list_sms(self, stat, mode=0):
        '''
        Returns an iterator object of SMS stored in memory. SMS same
        format with SimModule.read_sms.
        stat    ->  str. Status of the messages to read. Can be any
                    of the following:
                    REC UNREAD: Received unread messages
                    REC READ:   Received read messages
                    STO UNSENT: Stored unsent messages
                    STO SENT:   Stored sent messages
                    ALL:        All messages
        mode    ->  0:  Normal. Changes status of SMS record
                ->  1:  Not change status of the specified SMS
                        recode. e.g Unread messages will remain REC
                        UNREAD even after reading it.
        
        rtn format: ('index', 'stat', 'number', 'date', 'time', 'body')
        '''

        cmd = 'at+cmgl="{}",{}'.format(stat.upper(), mode)
        readbk = self.write_cmd(cmd=cmd)
        err_check(readbk, cmd)

        for i in range(0, len(readbk) - 2, 2):
            # readbk: (detail, content), increment by 2 to
            # treat detail and content as 1 item only
            # Remove " in the details string
            tmp = readbk[i].replace('"', '')
            # Convert the csv format detail string
            tmp = tmp.split(',')
            # Remove +CMGL: on the detail status: +CMGL: REC READ
            tmp[0] = tmp[0].replace('+CMGL: ', '')
            # Remove empty item on the list
            tmp = list(filter(None, tmp))
            # Append the message content
            tmp.append(readbk[i+1])
            # Yeild each item as SMS object
            yield tuple(tmp)

    def delete_sms(self, index=None, typ=None):
        '''
        Deletes message or group of message.
        index:  int. Index of message to be deleted
        typ:    str. Group type of messages to be deleted. typ 
                can be any of the following: 
                READ    -> Delete all read messages 
                UNREAD  -> Delete all unread messages
                SENT    -> Delete all sent messages   
                UNSENT  -> Delete all unsent messages
                INBOX   -> Delete all received messages
                ALL     -> Delete all messages 
        '''
        if index:
            logging.debug(msg='Deleting message at index {}'.format(index))
            cmd = 'at+cmgd={}'.format(index)
            readbk = self.write_cmd(cmd=cmd)
            logging.debug(msg='Delete result: {}'.format(' '.join(readbk)))
            err_check(readbk, cmd)
            return readbk[0]

        if typ:
            logging.debug(msg='Deleting messages with the type of {}'.format(typ))
            cmd = 'at+cmgda="{}"'.format('DEL ' + typ.upper())
            readbk = self.write_cmd(cmd=cmd)
            logging.debug(msg='Delete result: {}'.format(' '.join(readbk)))
            err_check(readbk, cmd)
            return readbk[0]

        return None

    ''' -------------------- GPRS Methods -------------------- '''
    @property
    def _gprs_last_used(self):
        with self._gprs_last_used_lock:
            return self.__gprs_last_used

    @_gprs_last_used.setter
    def _gprs_last_used(self, val):
        with self._gprs_last_used_lock:
            self.__gprs_last_used = val

    @property
    def gprs_service(self):
        logging.debug('Checking GPRS Service status')
        cmd = 'at+cgatt?'
        readbk = self.write_cmd(cmd=cmd)
        logging.debug('GPRS service stat: {}'.format(' '.join(readbk)))
        err_check(readbk, cmd)
        self._gprs_service = int(readbk[0].replace('+CGATT: ', ''))
        return self._gprs_service

    @gprs_service.setter
    def gprs_service(self, state):
        logging.debug('Setting GPRS Service status to {}'.format(state))
        cmd = 'at+cgatt={}'.format(state)
        readbk = self.write_cmd(cmd=cmd, read_timo=80)
        logging.debug('Command result: {}'.format(' '.join(readbk)))
        err_check(readbk, cmd)

    @property
    def gprs_profile(self):
        return self._gprs_profile
    
    @gprs_profile.setter
    def gprs_profile(self, profile):
        if not isinstance(profile, GPRSProfile):
            raise ValueError('{} is an naapropriate argument value for \'profile\''.format(profile))
        self._gprs_profile = profile

    def gprs_connect(self):
        logging.debug('Connecting module to GPRS network ...')

        with self.uart_lock:
            cmd = 'at+sapbr=3,{},"CONTYPE","GPRS"'.format(self.gprs_profile.cid)
            readbk = self.write_cmd(cmd=cmd)
            logging.debug('Set CONTYPE to GPRS: %s' %(' '.join(readbk)))
            err_check(readbk=readbk, cmd=cmd)

            cmd = 'at+sapbr=3,{},"APN","{}"'.format(self.gprs_profile.cid, self.gprs_profile.apn)
            readbk = self.write_cmd(cmd=cmd)
            logging.debug('Set APN to %s: %s' %(self.gprs_profile.apn, ' '.join(readbk)))
            err_check(readbk=readbk, cmd=cmd)

            cmd = 'at+sapbr=1,{}'.format(self.gprs_profile.cid)
            readbk = self.write_cmd(
                cmd=cmd,
                expected_response=('OK', 'CME ERROR'),
                read_timo=87
            )
            logging.debug('Connect result: %s' %(' '.join(readbk)))
            err_check(readbk=readbk, cmd=cmd, optional='GPRS already connected?')
            
            if self.gprs_inactive_timo:
                self._gprs_last_used = now()
                self.gprs_inactive_mon = InactiveMonitor(
                    target=None,
                    name='gprs_inactive_monitor',
                    kwargs={
                        'module'    : self,
                        'timeout'   : self.gprs_inactive_timo
                    }
                )
                self.gprs_inactive_mon.start()

        return readbk[0]

    def gprs_get_ip(self):
        cmd = 'at+sapbr=2,{}'.format(self.gprs_profile.cid)
        readbk = self.write_cmd(cmd=cmd)
        logging.debug('Getting Modules IP Address: %s' %(' '.join(readbk)))
        err_check(readbk=readbk, cmd=cmd)
        ip_regex = regex_compile(r'"(.*)"')
        ip = ip_regex.search(readbk[0])
        ip = ip.group(1)
        if not ip:
            raise TimeoutExpired('Unable to get valid IP address!')
        return ip

    def gprs_is_connected(self):
        
        ip = self.call_method(
            cmd=self.gprs_get_ip,
            max_try=5,
        )
        logging.debug(msg=ip)
        if ip == '0.0.0.0':
            return False
        return True

    def gprs_close(self):

        if self.gprs_inactive_timo:
            try:
                self.gprs_inactive_mon.stop()
            except AttributeError as e:
                logging.debug(msg=e.__str__())

        logging.debug('Closing GPRS connection ...')
        cmd = 'at+sapbr=0,{}'.format(self.gprs_profile.cid)

        with self.uart_lock:
            readbk = self.write_cmd(cmd=cmd, read_timo=70)
            logging.debug('GPRS close: %s' %(' '.join(readbk)))
            err_check(readbk=readbk, cmd=cmd, optional='GPRS already closed?')

            cmd = 'at+cipshut'
            readbk = self.write_cmd(
                cmd=cmd, 
                expected_response='SHUT OK',
                read_timo=10)
            logging.debug('CIPSHUT: %s' %(' '.join(readbk)))
            err_check(readbk=readbk, cmd=cmd)

        return readbk[0].replace('SHUT ', '')

    def tcp_connect(self, server):
        self._gprs_last_used = now()
        logging.debug('Connecting module to TCP server: %s: %s' %(server.domain, server.port))
        
        if not self.gprs_is_connected():                 # Prevent connecting to TCP server
            raise Exception('GPRS not connected.')   # if disconnected from GPRS network

        _resp = ('CONNECT OK', 'ALREADY CONNECT', 'CONNECT FAIL')
        cmd = 'at+cipstart="TCP","{}",{}'.format(server.domain, server.port)
        readbk = self.write_cmd(
            cmd=cmd,
            expected_response=_resp,
            read_timo=20
        )

        logging.debug('Connect result: %s' %(str(readbk)))
        if _resp[1] not in readbk:  # avoid false err_check trigger in 'ALREADY CONNECT' status
            err_check(readbk=readbk, cmd=cmd)
        # return readbk[1]

    def tcp_send(self, packet):
        self._gprs_last_used = now()
        logging.debug('Sending packet to TCP Server: %s' %packet)
        _resp = ('SEND OK', 'SEND FAIL', 'CME ERROR', 'ACCEPT', 'CLOSE')
        # cmd = 'at+cipsend={}'.format(len(packet))
        cmd = 'at+cipsend={}'.format(len(packet))

        with self.uart_lock:
            readbk = self.write_cmd(cmd=cmd, expected_response='>')
            logging.debug('readbk: {}'.format(readbk))
            if not readbk:  # Causes thread to block. Reset module to recover
                logging.error('expecting \'>\' but got empty readbk on send_tcp. Reset module to recover.')
                self.power_reset()
                self.initialize()
                readbk = () # let the caller handle the exception raised by err_check
            err_check(readbk=readbk, cmd=cmd)

            readbk = self.write_cmd(
                cmd=packet,
                expected_response=_resp,
                read_timo=10,
            )
            readbk = ''.join(readbk).strip()
            logging.debug('Send result: %s' %readbk)
            err_check(readbk=readbk, cmd=cmd)
            
        return readbk

    def tcp_status(self):
        _resp = ('IP INITIAL', 'CONNECT OK', 'TCP CLOSED')
        cmd = 'at+cipstatus'
        readbk = self.write_cmd(cmd=cmd, expected_response=_resp)
        err_check(readbk=readbk, cmd=cmd)
        
        # readbk = ('OK', '<status>')
        readbk = readbk[1].replace('STATE: ', '')
        return readbk

    def tcp_is_connected(self):
        if self.tcp_status() == 'CONNECT OK':
            return True
        return False

    def tcp_close(self):
        self._gprs_last_used = now()
        logging.debug('Closing TCP Server connection ...')
        cmd = 'at+cipclose'
        readbk = self.write_cmd(cmd=cmd, read_timo=5)
        logging.debug('Close result: %s' %(' '.join(readbk)))
        err_check(readbk=readbk, cmd=cmd, optional='TCP connection already closed?')
        return readbk[0]  # ('CLOSE OK',)

    def gsm_loc(self):
        if not self.gprs_is_connected():
            self.gprs_connect()

        self._gprs_last_used = now()
        logging.debug('Getting gsm based location ...')
        cmd = 'at+cipgsmloc=1,{}'.format(self.gprs_profile.cid)
        readbk = self.write_cmd(cmd=cmd, read_timo=60)
        logging.debug(' '.join(readbk))
        err_check(readbk=readbk, cmd=cmd)

        # get <data> on readbk. readbk = (<data>, <OK>)
        gsm_loc = readbk[0].split(',')
        lcode = gsm_loc[0].replace('+CIPGSMLOC: ', '')
        try:
            gps_data = GSMLocData(
                {
                    'lcode' : lcode,
                    'lng'   : gsm_loc[1],
                    'lat'   : gsm_loc[2],
                    'date'  : gsm_loc[3],
                    'time'  : gsm_loc[4],
                }
            )
        except IndexError as e:
            # logging.error(e.__str__())
            gps_data = GSMLocData(
                {
                    'lcode' : lcode
                }
            )
        
        gps_data.provider = 'NETWORK'
        return gps_data


class SIM808(SimModule):

    @staticmethod
    def parse_gprmc(gps_string):
        raw = gps_string.split(',')
        gprmc_dict = {
            'time'      :   raw[1], # Will be replaced by 'when' attribute
            'stat'      :   raw[2],
            'lat'       :   SimModule.dm_to_dd(raw[3]),
            'lng'       :   SimModule.dm_to_dd(raw[5]),
            'speed'     :   '%d kph' % (float(raw[7]) * 1.852),
            'course'    :   raw[8],
            'date'      :   raw[9],  # Will be replaced by 'when' attribute
        }

        return gprmc_dict

    def __init__(self, *args, **kwargs):
        super(SIM808, self).__init__(*args, **kwargs)

    '''  Support for GPS functionality '''
    @property
    def gps_pwr(self):
        cmd = 'at+cgpspwr?'
        readbk = self.write_cmd(cmd=cmd)
        logging.debug(msg=str(readbk))
        err_check(readbk, cmd)

        readbk = ' '.join(readbk)
        if '0' in readbk:
            self._gps_pwr = 0
        elif '1' in readbk:
            self._gps_pwr = 1
        else:
            self._gps_pwr = -1

        return self._gps_pwr

    @gps_pwr.setter
    def gps_pwr(self, val):
        if not isinstance(val, int):
            raise ValueError('argument "val" should be a type of int')
        if val > 1 or val < 0:
            raise ValueError('argument "val" should be 0 or 1 only')

        cmd = 'at+cgpspwr={}'.format(val)
        readbk = self.write_cmd(cmd=cmd)
        logging.debug(msg=str(readbk))
        err_check(readbk, cmd)

    @property
    def gps_rst(self):
        cmd = 'at+cgpsrst?'
        _resp = ('OK', 'ERROR')
        readbk = self.write_cmd(
            cmd=cmd,
            expected_response=_resp,
        )

        logging.debug(msg=str(readbk))
        err_check(readbk, cmd)

        readbk = ' '.join(readbk)
        if '0' in readbk:
            self._gps_rst = 0   # Cold
        elif '1' in readbk:
            self._gps_rst = 1   # Hot
        elif '2' in readbk:
            self._gps_pwr = 2  # Warm
        else:
            self._gps_rst = -1
        
        return self._gps_rst

    @gps_rst.setter
    def gps_rst(self, mode):
        if not isinstance(mode, int):
            raise ValueError('argument "val" should be a type of int')
        if mode > 1 or mode < 0:
            raise ValueError('argument "val" should be an integer value from 0 or 2 only')
        
        cmd = 'at+cgpsrst={}'.format(mode)
        _resp = ('OK', 'ERROR')
        readbk = self.write_cmd(cmd=cmd)
        logging.debug(msg=str(readbk))
        err_check(readbk, cmd)

    def gps_stat(self):
        cmd = 'at+cgpsstatus?'
        readbk = self.write_cmd(cmd=cmd)
        logging.debug(msg=str(readbk))
        err_check(readbk, cmd)
        
        readbk = ' '.join(readbk)
        gps_stat_regex = regex_compile(r'\+CGPSSTATUS: (.*) ')
        stat = gps_stat_regex.search(readbk)
        stat = stat.group(1)

        return stat

    def _gps_raw(self, inf):
        cmd = 'at+cgpsinf={}'.format(inf)
        readbk = self.write_cmd(cmd=cmd)
        logging.debug(msg=str(readbk))
        err_check(readbk=readbk, cmd=cmd, optional='Check GPS power.')

        return readbk[0]
            
    def gps_data(self, inf=32):
        readbk = self._gps_raw(inf=inf)
        readbk = readbk.replace('+CGPSINF: ', '')

        if inf == 32:
            gps_data = GPRMCData(
                gps_data=SIM808.parse_gprmc(gps_string=readbk)
            )
            gps_data.provider = 'GPS'

        return gps_data


class GPRSProfile():
    def __init__(self, cid, apn, user='', pw=''):
        self.cid = cid
        self.apn = apn
        self.user = user
        self.pw = pw

    @property
    def ip_address(self):
        return self._ip_address

    @ip_address.setter
    def ip_address(self, ip):
        self._ip_address = ip

    @property
    def stats(self):
        return self._status

    @stats.setter
    def stats(self, value):
        self._status = value


class TCPServer():
    def __init__(self, domain, port):
        self.domain = domain
        self.port = port

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, status):
        self._status = status


class SMS():
    pass


class GPSData():
    # Always check is_valid first before working with the gps objects
    
    def __init__(self, gps_data):
        for k, v in gps_data.items():
            setattr(self, k, v)
        
        # Will have a value if is_valid = True
        self.when = None
        self.timestamp = None

    def __str__(self):
        return self.__dict__.__str__()

    def is_valid(self, gps_data):
        pass

    @property
    def when(self):
        ''' return string UTC time '''
        return self._when.strftime('%c %Z')
    
    @when.setter
    def when(self, dt):
        ''' dt: datetime instance formatted from GPS data '''
        self._when = dt

class GPRMCData(GPSData):
    def __init__(self, gps_data, *args, **kwargs):
        super(GPRMCData, self).__init__(gps_data=gps_data)
        self.is_valid = self.stat
        self.__delattr__('stat') # remove stat. replaced by is_valid

    @property
    def is_valid(self):
        return self._is_valid
    
    @is_valid.setter
    def is_valid(self, stat):
        if stat == 'A':
            self._is_valid = True
            self.time = self.time.split('.')[0]
            self.when = utc.localize(
                datetime.strptime(
                    '{} {}'.format(self.date, self.time),
                    '%d%m%y %H%M%S'
                )
            )
            self.timestamp = self._when.timestamp()
            # Delete 'date' and 'time' attributes
            self.__delattr__('date')
            self.__delattr__('time')

        else:
            self._is_valid = False
            self.__delattr__('when')

        
class GSMLocData(GPSData):
    def __init__(self, gps_data, *args, **kwargs):
        super(GSMLocData, self).__init__(gps_data)
    
    @property
    def is_valid(self):
        return self._is_valid
    
    @is_valid.setter
    def is_valid(self, lcode):
        if lcode != '0':
            self._is_valid = False
            self.__delattr__('when')
        else:
            self._is_valid = True
            self.when = utc.localize(
                datetime.strptime(
                    '{} {}'.format(self.date, self.time),
                    '%Y/%m/%d %H:%M:%S %z'
                )
            )
            self.timestamp = self.when.timestamp()
            # Delete 'date' and 'time' attributes
            self.__delattr__('date')
            self.__delattr__('time')


class CommandError(Exception):
    def __init__(self, error, cmd, optional=''):
        super(CommandError, self).__init__(
            '{} during {} excecution. {}'.format(error, cmd, optional)
        )

class ModuleResponseError(Exception):
    ''' Raised when write_cmd raises UnicodeDecodeError. Experienced
    when LM2596 DC-to-DC converter was placed near the PI. '''
    def __init__(self, error, resp, optional=''):
        super(ModuleResponseError, self).__init__(
            '{} during {} readback. {}'.format(error, resp, optional)
        )


class TimeoutExpired(Exception):
    def __init__(self, cmd):
        super(TimeoutExpired, self).__init__(
            'Timeout expired, got no response from the module on {} command.'.format(cmd)
        )

""" -------------------- Test Codes ---------------------- """

# s = SIM808(port='COM4')
# s.gprs_profile = GPRSProfile(cid=1, apn='internet.globe.com.ph')
# ubidots = TCPServer('things.ubidots.com', 9012)

# def close_network_connection():

#     # if connected, close gprs connection
#     if s.gprs_is_connected():
#         logging.debug('Closing GPRS')
#         try:
#             s.gprs_close()
#         except CommandError as e:
#             logging.error(msg=e.__str__())

#     if s.tcp_is_connected():
#         logging.debug('Closing TCP')
#         try:
#             s.tcp_close()
#         except CommandError as e:
#             logging.error(msg=e.__str__())

# tcp_packet = '''SIM808_test|POST|A1E-YAmgm46yp3XrCtoYWou3dd8pcwfGYU|\
# raspberrypi=>temp:{}|end'''.format(randint(24, 27))

# def initialize():
#     # Close any existing connection
#     close_network_connection()


# if __name__ == '__main__':

#     pass

    # index = 1

    # while True:
    #     msg = s.read_sms(index=index)
    #     if not msg:
    #         break
    #     for _ in msg:
    #         print(_)
    #     print()
    #     index += 1


    # import time

    # t1 = time.time()
    # loop_count = 0
    # while True:
    #     duration = time.time() - t1
    #     loop_count += 1

    #     logging.debug(msg='Loop count: {} | TCP status: {} | Duration: {}'
    #                   .format(loop_count, s.tcp_status(), duration))

    #     try:
    #         s.tcp_connect(ubidots)
    #     except CommandError as e:
    #         logging.error(msg=e.__str__())
        
    #     delay(2.5)

    #     try:
    #         s.tcp_send(tcp_packet)
    #     except CommandError as e:
    #         logging.error(msg=e.__str__())
        
    #     delay(2.5)

    #     try:
    #         s.tcp_close()
    #     except CommandError as e:
    #         logging.error(msg=e.__str__())
        
    #     print('\n')
    #     delay(5)
    
    # logging.error(msg='Total duration: {}'.format(time.time() - t1))
