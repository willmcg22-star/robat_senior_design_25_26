"""
This module provides utility functions interfacing with the motor communication protocol.
"""

import time

import serial
from serial import Serial
from enum import Enum

def crc16(data: bytes):
    """
    A utility function for computing CRC-16 (CCITT) implemented with a precomputed lookup table

    :param data: an array of bytes to compute crc 16 on
    :type data: bytes
    :return: an array of 2 bytes with the first element being the high bytes, and the second being the low bytes
    :rtype: bytes
    """
    table = [ 0x0000,
        0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
        0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
        0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
        0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
        0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
        0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
        0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
        0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
        0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
        0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
        0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
        0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
        0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
        0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
        0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
        0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
        0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
        0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
        0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
        0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
        0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
        0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
        0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
        0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
        0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
        0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
        0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
        0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
        0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
        0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
        0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
        0x820D, 0x8207, 0x0202 
    ]
    
    crc = 0x0000
    for byte in data:
        crc = (crc << 8) ^ table[(crc >> 8) ^ byte & 0xFF]
        crc &= 0xFFFF                                   # important, crc must stay 16bits all the way through

    crc_h = crc >> 8
    crc_l = crc & 0xFF
    return [crc_h, crc_l]

class OPCODE(Enum):
    """
    A data type for enumerating the opcodes for the tendon communication protocol. Note that the enum values
    correspond to the actual opcode value in the protocol.
    """
    ECHO = 0
    READ_STATUS = 1
    READ_ANGLE = 2
    WRITE_ANGLE = 3
    WRITE_PID = 4

class TendonHardwareInterface:
    """
    This module is designed acts as an abstraction layer between a high level Tendoncontrol API and the motor communication protocol.
    This class should not be instantiated directly for any user code.
    This class contains functions responsible for assembling and unpacking packet data transmitted to and from the motor controller.

    Upon initialization, a serial connection is opened with the device specified by ``port_name``. If ``port_name``, is ``NONE``, then no serial connection is opened.
    The serial device is closed automatically upon destruction.

    :param port_name: The serial port name used for communication with the motor controller
    :type port_name: str
    :ivar ser: An instance of the serial communication device
    :vartype ser: pyserial.Serial
    :ivar packet: A byte array storing the packet to be transmitted or received
    :vartype packet: byte
    """

    def __init__(self, port_name):
        self.ser = None
        if port_name:
            self.ser = Serial(port_name, baudrate=115200, parity=serial.PARITY_NONE, stopbits=1)
        
        self.packet = []

    def __del__(self):
        self.ser.close()
        print("Terminated serial connection")

    def BuildPacket(self, id, opcode, params):
        """
        This function constructs a packet according to the the motor communication protocol
        and automatically appends a CRC16 checksum to the end of the packet.
        The packet is stored in the ``packet`` instance variable and NOT returned to the user.
        This function should therefore be called before everytime a packet is sent.
        This function does NOT check:

        - if the id is valid
        - if the opcode is valid

        :param id: The id of the motor to be commanded
        :type id: int
        :param opcode: The opcode corresponding to the operation to perform on motor ``id``
        :type opcode: int
        :param params: An array of packet parameters for the motor operation
        :type params: list[int]
        """

        data = [0xFF, 0x00]

        length = len(params) + 4
        data.append(length)
        data.append(id)
        data.append(opcode)
        data = data + params
        crc = crc16(data)
        data = data + crc

        self.packet = data

    def ReadRx(self):
        """
        This function reads a packet from the serial device. 
        Because the motor communication protocol function is a request-response model, this function shouldn't
        be directly called by user code. Instead, this function is a helper function for the ``SendTxRX`` function.

        This function will return a timeout error if no valid packets are ever read. The error will be given as the function returning -1 (TODO: its preferrable to raise an exception instead).
        However, the function will block if no serial data is ever received (TODO: need to fix that and set a timeout). 
        This function also performs automatic CRC checking, but will still return the data if CRC validation fails (TODO: maybe raise an exception instead).
        Otherwise, if a packet is successfully read, the function returns a byte array containing everything after the packet header.

        :return: A byte array containing the received packet data (without the packet header) or -1 if any communication errors occured
        :rtype: bytes or int
        """
        data = list(self.ser.read(2))

        timeout = 5000          # TODO: make this a instance variable that can be set
        start = time.time()

        while data != [0xff, 0x00]:
            end = time.time()

            if 1000*(end - start) > timeout:
                self.ser.flush()
                print("Timeout error")  # Make this an exception
                return -1

            data[0] = data[1]
            data[1] = int.from_bytes(self.ser.read(1), byteorder='big')

        len = int.from_bytes(self.ser.read(1), byteorder='big')
        data.append(len)
        for i in range(0, len):
            byte = int.from_bytes(self.ser.read(), byteorder='big')
            data.append(byte)

        self.ser.reset_input_buffer()

        crc = data[-2:]
        data = data[0:-2]

        new_crc = crc16(data)

        if crc != new_crc:
            print('CRC ERROR!')
            return data

        return data

    def SendTxRx(self):
        """
        This function sends and reads a packet from the serial device. 
        This function should be called directly in user code.
        Before calling this function be sure to call :func:`~batbot_bringup.bb_tendons.TendonHardwareInterface.BuildPacket` to set the packet to be sent.
        Read :func:`~batbot_bringup.bb_tendons.TendonHardwareInterface.ReadRxx` for information on possible errors when reading packets.
        If any errors occured, this function will return -1 (TODO: maybe raise an exception instead). If the serial transaction was successfull, then
        a dictionary will be returned with the following fields:

        - *id*: the id of the motor being commanded
        - *opcode*: the opcode corresponding to the motor command sent
        - *status*: the status bit returned by the motor operation (refer to :ref:`tendon-embedded-software` for status codes)
        - *params*: the parameters returned by the response packet

        :return: A dict containing the response packet data as described above or -1 if any communication errors occured
        :rtype: dict or int
        """
        self.SendTx()

        data = self.ReadRx()

        if data != -1:
            return {
                "id": data[3],
                "opcode": data[4],
                "status": data[5],
                "params": data[6:]
            }
        else:
            return -1

    def SendTx(self):
        """
        This function sends a packet from the serial device without reading the response packet. 
        This function can be called directly in user code, but please make sure to clear the serial buffer to ensure future packets are read properly.
        Before calling this function be sure to call :func:`~batbot_bringup.bb_tendons.TendonHardwareInterface.BuildPacket` to set the packet to be sent.
        """
        self.ser.reset_output_buffer()
        self.ser.write(bytes(self.packet))