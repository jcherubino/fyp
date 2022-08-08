'''
UART interface to interact and read data from DWM1001-Dev flashed with custom firmware (user application on top of PANs stack) by reading printed lines of serial data.

Written by Josh Cherubino (josh.cherubino@gmail.com)
'''
import sys
import serial
import enum
import logging
import logging.config
from logger_config import LOG_DICT_CONFIG

logger = logging.getLogger(__name__)

class TagInterface:
    '''
    Tag interface class to manage communication of data
    '''

    def __init__(self, port):
        self.conn = serial.Serial(port, baudrate=115200)
        logger.info('Initialised Tag connection on port %s', port)

    def reset(self):
        self.conn.reset_input_buffer()
        self.conn.readline()

    def shutdown(self):
        self.conn.close()
        logger.info('Shutdown Tag connection')

    def read_data(self):
        data = self.conn.readline().decode('utf-8')
        #logger.debug(data)
        fall, px, py, qf, ax, ay, az = data.split(',')
        fall, px, py, qf, ax, ay, az = bool(fall), int(px), int(py), int(qf),\
                int(ax), int(ay), int(az)
        return fall, px, py, qf, ax, ay, az    

if __name__ == '__main__':
    logging.config.dictConfig(LOG_DICT_CONFIG)
    logger = logging.getLogger(__name__)
    port = sys.argv[1]
    interface = TagInterface(port) 
    while True:
        interface.read_data()
    interface.shutdown()
