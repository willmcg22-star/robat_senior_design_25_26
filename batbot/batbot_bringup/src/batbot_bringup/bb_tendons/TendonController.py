'''
This module provides high level functions for interacting with the motor communication protocol.

After following the :ref:`software-setup`, you should be able to import this module as follows:

::

    import batbot_bringup.bb_tendons.TendonController

See the rest of this documentation for the available classes and functions in this module.
'''

from ..bb_tendons.TendonHardware import TendonHardwareInterface
import time
import struct
import numpy as np

from enum import Enum

class COM_TYPE(Enum):
    """
    A data type for enumerating the communication types available for the TendonController.
    """
    NONE = -1
    SPI = 0
    FAKE_SPI = 1
    UART = 2

class OPCODE(Enum):
    """
    A data type for enumerating the opcodes for the tendon communication protocol. Note that the enum values
    correspond to the actual opcode value in the protocol. (TODO: Probably delete this and use the one in TendonHardware.py)
    """
    ECHO = 0
    READ_STATUS = 1
    READ_ANGLE = 2
    WRITE_ANGLE = 3
    WRITE_PID = 4
    SET_ZERO_ANGLE = 5
    SET_MAX_ANGLE = 6


class TendonController:
    '''
    This class is used to control and interface with NEEC motor controller via a
    serial connection.

    Built off the :class:`~batbot_bringup.bb_tendons.TendonHardwareInterface` class for packing and unpacking serial packet data.'
    
    :param com: An enum specifying the communication interface to use (UART, SPI, etc.) (NOT USED!)
    :type com: COM_TYPE
    :param port_name: The serial port name used for communication with the motor controller
    :type port_name: str
    :ivar test_mode: A boolean specifying if the tendon controller is in test mode
    :vartype test_mode: bool
    :ivar test__angle: An internal variable used to store the angle in test mode
    :vartype test__angle: int
    :ivar test__max_angle: A internal variable used to store the max angle in test mode
    :vartype test__max_angle: int
    :ivar th: An instance of the :class:`~batbot_bringup.bb_tendons.TendonHardwareInterface` class to be used as a packet handler
    :vartype th: :class:`~batbot_bringup.bb_tendons.TendonHardwareInterface`

    HOW TO USE
    ==========
    1) First, create a TendonController object specifying the communication
    type and port number. Example:

    ::
    
        tendonController = TendonController(port_name="/dev/ttyACM0")

    If no port name is specified, then the TendonController is started in test mode. In test mode, any commands are simply
    written or read from internal variables.

    2) You can then call any function to control a motor. For example,
    to write motor 0 to 120 degrees call:

    ::
    
        tendonController.writeMotorAngle(0, 120)

    If this file is run as a script, it will run an example program utilizing this class.
    '''


    def __init__(self, com=COM_TYPE.NONE, port_name=''):

        # The following variables are used if no device is connected
        self.test_mode = True
        self.test__angle = 0
        self.test__max_angle = 0

        print(port_name)

        if port_name != '':
            self.th = TendonHardwareInterface(port_name)
            self.test_mode = False
        else:
            print("WARNING: Beginning tendon calibration in test mode! Please supply a port name if this wasn't intentional.")
            time.sleep(3)

    def writeMotorAbsoluteAngle(self, id, angle: np.int16):
        '''
        This function sets the motor specified by id to move to the passed in angle.
        Raises an assertion error if the status returned by the response packet is not ``COMM_SUCCESS``

        TODO: The function currently breaks if any communication errors occured during packet transmission

        In test mode, the angle value simply gets written to ``test__angle``.

        :param id: The motor id whose angle is to be set
        :type id: int
        :param angle: The motor angle
        :type opcode: signed int16
        '''
        
        if not self.test_mode:
            angle_h = (angle >> 8) & 0xFF
            angle_l = (angle & 0xFF)

            params = [angle_h, angle_l]

            self.th.BuildPacket(id, OPCODE.WRITE_ANGLE.value, params)
            ret = self.th.SendTxRx()

            assert(ret["status"] == 0)
        else:
            self.test__angle = angle

    def readMotorAngle(self, id):
        '''
        This function reads the motor specified by id.
        Raises an assertion error if the status returned by the response packet is not ``COMM_SUCCESS``

        TODO: The function currently breaks if any communication errors occured during packet transmission

        In test mode, the angle value simply reads the value of ``test__angle``.

        :param id: The motor id whose angle is to be set
        :type id: int
        :return: The angle of motor ``id``
        :rtype: signed int16
        '''

        if not self.test_mode:
            self.th.BuildPacket(id, OPCODE.READ_ANGLE.value, [])
            ret = self.th.SendTxRx()

            if (self.test_mode):
                return 0

            if ret != -1:
                assert(ret["status"] == 0)


                angle = np.int16(((ret["params"][0])  << 8) | (ret["params"][1] & 0xFF))
                return angle
        else:
            return self.test__angle

    def moveMotorToMin(self, id):
        '''
        NOT USED!
        '''
        self.writeMotorAnglePercentMax(id, 0)

    def moveMotorToMax(self, id):
        '''
        NOT USED!
        '''
        self.writeMotorAnglePercentMax(id, 100)

    def setNewZero(self, id):
        '''
        Resets the encoder count of the motor specified by id, effectively setting the current position to the 0 angle.
        Raises an assertion error if the status returned by the response packet is not ``COMM_SUCCESS``

        TODO: The function currently breaks if any communication errors occured during packet transmission

        In test mode, the ``test__angle`` variable is simply set to 0.

        :param id: The motor id whose angle is to be reset
        :type id: int
        '''

        if not self.test_mode:
            params = []
            
            self.th.BuildPacket(id, OPCODE.SET_ZERO_ANGLE.value, params)
            ret = self.th.SendTxRx()

            assert(ret["status"] == 0)
        else:
            self.test__angle = 0
    
    def setMotorMaxAngle(self, id, angle):
        '''
        This function sets the max angle of the motor specified by id to the passed in angle.
        Raises an assertion error if the status returned by the response packet is not ``COMM_SUCCESS``

        TODO: The function currently breaks if any communication errors occured during packet transmission

        In test mode, the angle value simply gets written to ``test__max_angle``.

        :param id: The motor id whose angle is to be set
        :type id: int
        :param angle: The maximum motor angle
        :type opcode: signed int16
        '''
        if not self.test_mode:
            angle_h = (angle >> 8) & 0xFF
            angle_l = angle & 0xFF

            params = [angle_h, angle_l]

            self.th.BuildPacket(id, OPCODE.SET_MAX_ANGLE.value, params)
            ret = self.th.SendTxRx()

            assert(ret["status"] == 0)
        else:
            self.test__max_angle = angle

    def setMotorPID(self, id, Kp, Ki, Kd):
        """
        Sets the PID (Proportional-Integral-Derivative) parameters for a motor.
        """
        if not self.test_mode:
        # Convert each PID parameter from float64 (default in Python) to float32 (4-byte representation)
            kp_bytes = struct.pack('>f', float(Kp))  # 'f' specifies a 32-bit float, '>' specifies big endian
            ki_bytes = struct.pack('>f', float(Ki))
            kd_bytes = struct.pack('>f', float(Kd))
        
        # Convert the byte sequences into lists of individual byte values and combine them
            params = list(kp_bytes) + list(ki_bytes) + list(kd_bytes)
            print(params)
        
            self.th.BuildPacket(id, OPCODE.WRITE_PID.value, params)
            ret = self.th.SendTxRx()

            assert(ret["status"] == 0)
        else:
            print(f"Test mode: Setting PID parameters for motor {id}: Kp={Kp}, Ki={Ki}, Kd={Kd}")
            
    def close(self):
        if self.th.ser:
            self.th.ser.close()

if __name__ == "__main__":  

    import time

    tc = TendonController(port_name="COM3")

    # tc.setMotorMaxAngle(0, 180)

    angles = [0, 0, 0, 0, 0]

    while True:
        for i in range(0, 5):
            angles[i] = tc.readMotorAngle(i)

        print(angles)

        time.sleep(3)
            
            