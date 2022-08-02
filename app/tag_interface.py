'''
UART interface to interact and read data from DWM1001-Dev flashed with custom firmware (user application on top of PANs stack) via the Type Length
Value (TLV) communication scheme

Written by Josh Cherubino (josh.cherubino@gmail.com)
'''
import sys
import serial
import enum
import struct
import logging
import logging.config
from typing import Optional
from logger_config import LOG_DICT_CONFIG

logger: Optional[logging.Logger] = None

class TagInterface:
    '''
    Tag interface class to manage communication of data
    '''
    TLV_RET_VAL_TYPE = 0x40
    RETURN_TLV_LENGTH = 3
    TLV_DWM_POS_GET = bytearray([0x02, 0x00])
    TLV_DWM_LABEL_READ = bytearray([0x1c, 0x00])

    class ErrorCode(enum.IntEnum):
        OK = 0
        BROKEN_OR_UNKNOWN_TLV = 1
        INTERNAL_ERROR = 2
        INVALID_PARAMETER = 3
        BUSY = 4
        OP_NOT_PERMITTED = 5

    def __init__(self, port):
        self.conn = serial.Serial(port, baudrate=115200)
        logger.info('Initialised Tag connection on port %s', port)

    def shutdown(self):
        self.conn.close()
        logger.info('Shutdown Tag connection')

    def dwm_pos_get(self):
        typ, length, value = self.request(self.TLV_DWM_POS_GET)
        # N.B. assume that type and length are appropriate values.
        x_mm, y_mm, z_mm, quality = struct.unpack('<iiiB', value) 
        return x_mm, y_mm, z_mm, quality
 
    def dwm_label_read(self):
        typ, length, value = self.request(self.TLV_DWM_LABEL_READ)
        label = value.decode('utf-8')
        logger.info('dwm_label_read: %s', label)
        return label

    def request(self, tlv_req: bytearray):
        self.conn.write(tlv_req)
        logger.debug('TLV request sent to tag')
        # validate that request was successful
        try:
            self.parse_return_tlv(self.conn.read(self.RETURN_TLV_LENGTH))
        except Exception as e:
            logger.exception('Exception occurred while validating TLV return header')
            # for now propagate error
            raise e
        
        logger.debug('TLV request acknowledged')
     
        # read actual response. First read 2 bytes to get type and value
        header = self.conn.read(2)
        typ, length = struct.unpack('<BB', header)
        value = self.conn.read(length)
        return typ, length, value

    def parse_return_tlv(self, ret_tlv: bytearray):
        # Returned TLV has type 0x40, length 1 and an error code.
        # First validate that ret_tlv is actually a returned TLV
        typ, length, value = struct.unpack('<BBB', ret_tlv)
        if typ != self.TLV_RET_VAL_TYPE:
            raise ValueError(f'TLV is not a returned TLV frame. TLV has type {typ}')
        if length != 1:
            raise ValueError(f'Returned TLV has invalid length: {length}')
        
        # Then check error code
        if value != self.ErrorCode.OK:
            raise ValueError(f'Request failed with error code: {self.ErrorCode(value).name}')

if __name__ == '__main__':
    logging.config.dictConfig(LOG_DICT_CONFIG)
    logger = logging.getLogger(__name__)
    port = sys.argv[1]
    interface = TagInterface(port) 
    interface.dwm_label_read()
    while True:
        print(interface.dwm_pos_get())
    interface.shutdown()
