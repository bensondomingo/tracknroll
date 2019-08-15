from serial import Serial
from sys import stdout as sys_stdout
import logging, os

base_dir = os.path.dirname(os.path.abspath(__file__))
log_file = os.path.join(base_dir, 'gps_test.log')
test_file = os.path.join(base_dir, 'test_fie.txt')

# Create log file if not exists
if 'gps_test.log' not in base_dir:
    with open(log_file, 'w') as f:
        pass

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file, 'a'),
        logging.StreamHandler(sys_stdout)
    ]
)

gps_uart = Serial('/dev/ttyS0', timeout=2)

def main():
    with open(test_file, 'w') as f:
        while True:
            logging.debug('reading gps_uart ...')
            gps_data = gps_uart.read()
            logging.info('gps_data: %s' %gps_data)

            if not gps_data:
                logging.error('empty!')
                continue

            logging.info('writing gps_data ...')
            f.write(gps_data.decode())

if __name__ == '__main__':
    main()