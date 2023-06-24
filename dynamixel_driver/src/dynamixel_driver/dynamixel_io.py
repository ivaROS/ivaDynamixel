#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Cody Jorgensen, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__author__ = "Cody Jorgensen, Antons Rebguns"
__copyright__ = "Copyright (c) 2010-2011 Cody Jorgensen, Antons Rebguns"

__license__ = "BSD"
__maintainer__ = "Antons Rebguns"
__email__ = "anton@email.arizona.edu"


import time
import serial
import struct
from array import array
from threading import Lock

from .dynamixel_const import *
from .robotis_def import *

exception = None


class DynamixelIO(object):
    """
    Provides low level IO with the Dynamixel servos through pyserial. Has the
    ability to write instruction packets, request and read register value
    packets, send and receive a response to a ping packet, and send a SYNC WRITE
    multi-servo instruction packet.
    """

    def __init__(self, port, baudrate, readback_echo=False):
        """Constructor takes serial port and baudrate as arguments."""
        try:
            self.serial_mutex = Lock()
            self.ser = None
            self.ser = serial.Serial(port, baudrate, timeout=0.015)
            self.port_name = port
            self.readback_echo = readback_echo
        except SerialOpenError:
            raise SerialOpenError(port, baudrate)

    def __del__(self):
        """Destructor calls DynamixelIO.close"""
        self.close()

    def close(self):
        """
        Be nice, close the serial port.
        """
        if self.ser:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.close()

    def __print_packet(self, packet):
        print(self.__packet2str(packet))    # Note: print with endline

    def __packet2str(self, packet):
        result = ""
        for ii in range(len(packet)):
          result += "{:02x} ".format(packet[ii])
        
        return result

    def __write_serial(self, packet):
        data = array("B", packet)
        self.ser.flushInput()
        self.ser.flushOutput()
        self.ser.write(data)
        if self.readback_echo:
            self.ser.read(len(data))

    # Cut-and-pasted from protocol2_packet_handler.py
    def __update_crc(self, crc_accum, data_blk_ptr, data_blk_size):
        """
        Function cut-and-pasted from ROBOTIS code base.
          Git repository: https://github.com/ROBOTIS-GIT/DynamixelSDK
          File: protocol2_packet_handler.py
        """
        crc_table = [0x0000,
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
                     0x820D, 0x8207, 0x0202]

        for j in range(0, data_blk_size):
            i = ((crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF
            crc_accum = ((crc_accum << 8) ^ crc_table[i]) & 0xFFFF

        return crc_accum

    def __read_response(self, servo_id):
        data = []

        try:
            # read the header to check how many more bytes to read
            buf = self.ser.read(7)
            data += struct.unpack("<7B", buf)

            # throw an exception if we have the wrong number of bytes
            if len(data) < 7:
                raise Exception("Wrong packet preamble length %d (preamble: %s)" % (len(data), data))

            # throw an exception if expected header not recieved
            if data[:4] != [0xFF, 0xFF, 0xFD, 0x00]:
                raise Exception("Wrong packet prefix %s" % data[:4])

            # now read the rest of the packet
            n = DXL_MAKEWORD(data[5], data[6])  # -> data[PKT_LENGTH_L], data[PKT_LENGTH_H]
            
            buf = self.ser.read(n)
            data += struct.unpack("<{}B".format(n), buf)
        except Exception as e:
            raise DroppedPacketError(
                "Invalid response received from motor %d. %s" % (servo_id, e)
            )

        # verify checksum
        checksum_rcvd = DXL_MAKEWORD(data[-2], data[-1])
        checksum_recomputed = self.__update_crc(0, data[0:-2], len(data) - 2)
        if not checksum_rcvd == checksum_recomputed:
            raise ChecksumError(servo_id, data, checksum_rcvd)

        return data

    def read(self, servo_id, address, size):
        """
        Read "size" bytes of data from servo with "servo_id" starting at the
        register with "address". It is recommended to use the constants in module 
        dynamixel_const for readability.

        To read the position from servo with id 1, the method should be called
        like:
            read(1, DXL_GOAL_POSITION, 4)
        """
        # Data block length: # of bytes following standard header (0xFF, 0xFF, 0xFD, 0x00, id, length_l, length_h)
        length = 7  # instruction, address_l, address_h, size_l, size_h, crc_l, crc_h

        # Data block
        packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, DXL_LOBYTE(length), DXL_HIBYTE(length), DXL_READ_DATA]
        packet.extend([DXL_LOBYTE(address), DXL_HIBYTE(address)])
        packet.extend([DXL_LOBYTE(size), DXL_HIBYTE(size)])

        # Compute checksum from data block
        checksum = self.__update_crc(0, packet, len(packet))

        # Finish packet construction: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet.extend([DXL_LOBYTE(checksum), DXL_HIBYTE(checksum)])
        #self.__print_packet(packet)  # [DEBUG]

        with self.serial_mutex:
            self.__write_serial(packet)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)  # 0.00235)

            # read response
            data = self.__read_response(servo_id)
            data.append(timestamp)    # [TODO] not used by any other code in Dynamixel motor_stack; remove in future update

        return data

    def write(self, servo_id, address, data):
        """
        Write the values from the "data" list to the servo with "servo_id"
        starting with data[0] at "address", continuing through data[n-1] at
        "address" + (n-1), where n = len(data). It is recommended to use the 
        constants in module dynamixel_const for readability. "data" is a 
        list/tuple of integers.

        To set servo with id 1 to position 276, the method should be called
        like:
            write(1, DXL_GOAL_POSITION, (20, 1, 0, 0))
        """
        # Data block length: # of bytes following standard header (0xFF, 0xFF, 0xFD, 0x00, id, length_l, length_h)
        length = 5 + len(data)  # instruction, address_l, address_h, len(data), checksum_l, checksum_h

        # Data block
        packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, DXL_LOBYTE(length), DXL_HIBYTE(length), DXL_WRITE_DATA]
        packet.extend([DXL_LOBYTE(address), DXL_HIBYTE(address)])
        packet.extend(data)

        # Compute and append checksum bytes
        checksum = self.__update_crc(0, packet, len(packet))
        packet.extend([DXL_LOBYTE(checksum), DXL_HIBYTE(checksum)])

        #self.__print_packet(packet)     # [DEBUG]

        with self.serial_mutex:
            self.__write_serial(packet)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)

            # read response
            data = self.__read_response(servo_id)
            data.append(timestamp)    # [TODO] not used by any other code in Dynamixel motor_stack; remove in future update

        return data

    def sync_write(self, address, data):
        """
        Use Broadcast message to send multiple servos instructions at the
        same time. No "status packet" will be returned from any servos.
        It is recommended to use the constants in module dynamixel_const for 
        readability. "data" is a tuple of tuples. Each tuple in "data" must 
        contain the servo id followed by the data that should be written 
        from the starting address. The amount of data can be as long as needed.

        To set servo with id 1 to position 276 and servo with id 2 to position
        550, the method should be called like:
            sync_write(DXL_GOAL_POSITION, ( (1, 20, 1, 0, 0), (2 ,38, 2, 0, 0) ))
        """
        # Data: list of tuples -> each tuple = (servo_id, data1, data_2, ...)
        flattened = [value for servo in data for value in servo]

        # Data block length: # of bytes following standard header (0xFF, 0xFF, 0xFD, 0x00, id, length_l, length_h)
        # i.e., instruction, address_l, address_h, data_len_l, data_len_h, len(flattened), checksum_l, checksum_h
        length = 7 + len(flattened) 

        # Data block
        packet = [0xFF, 0xFF, 0xFD, 0x00, DXL_BROADCAST, DXL_LOBYTE(length), DXL_HIBYTE(length), DXL_SYNC_WRITE]
        packet.extend([DXL_LOBYTE(address), DXL_HIBYTE(address)])
        servo_data_len = len(data[0][1:])   # per servo data len
        packet.extend([DXL_LOBYTE(servo_data_len), DXL_HIBYTE(servo_data_len)])
        
        packet.extend(flattened)
        
        # Compute and append checksum bytes
        checksum = self.__update_crc(0, packet, len(packet))
        packet.extend([DXL_LOBYTE(checksum), DXL_HIBYTE(checksum)])

        #self.__print_packet(packet)

        with self.serial_mutex:
            self.__write_serial(packet)

    def ping(self, servo_id): 
        """
        Ping the servo with "servo_id". This causes the servo to return a
        "status packet". This can tell us if the servo is attached and powered,
        and if so, if there are any errors.
        """
        # Data block length: # of bytes following standard header (0xFF, 0xFF, 0xFD, 0x00, id, length_l, length_h)
        length = 3  # instruction, checksum_l, checksum_h

        # Data block
        packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, DXL_LOBYTE(length), DXL_HIBYTE(length), DXL_PING]

        # Compute and append checksum bytes
        checksum = self.__update_crc(0, packet, len(packet))
        packet.extend([DXL_LOBYTE(checksum), DXL_HIBYTE(checksum)])

        with self.serial_mutex:
            self.__write_serial(packet)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)

            # read response
            try:
                response = self.__read_response(servo_id)
                response.append(timestamp)    # [TODO] not used by any other code in Dynamixel motor_stack; remove in future update
            except Exception as e:
                response = []

        if response:
            self.exception_on_error(response[8], servo_id, "ping")
        return response

    def test_bit(self, number, offset):
        """
        Evalute value at bit position of "number", where "offset" = 0 
        corresponds to LSB
        """
        mask = 1 << offset
        return number & mask

    ######################################################################
    # These functions modify EEPROM data which persists after power cycle #
    ######################################################################

    def set_id(self, old_id, new_id):
        """
        Sets a new unique number to identify a motor. The range from 1 to 253
        (0xFD) can be used.
        """
        response = self.write(old_id, DXL_ID, [new_id])
        if response:
            self.exception_on_error(response[8], old_id, "setting id to %d" % new_id)
        return response

    def set_baud_rate(self, servo_id, baud_rate):
        """
        Sets servo communication speed. The range from 0 to 254.
        """
        response = self.write(servo_id, DXL_BAUD_RATE, [baud_rate])
        if response:
            self.exception_on_error(
                response[8], servo_id, "setting baud rate to %d" % baud_rate
            )
        return response

    def set_return_delay_time(self, servo_id, delay):
        """
        Sets the delay time from the transmission of Instruction Packet until
        the return of Status Packet. 0 to 254 (0xFE) can be used, and the delay
        time per data value is 2 usec.
        """
        response = self.write(servo_id, DXL_RETURN_DELAY_TIME, [delay])
        if response:
            self.exception_on_error(
                response[8], servo_id, "setting return delay time to %d" % delay
            )
        return response

    def set_angle_limit_min(self, servo_id, min_angle):
        """
        Set the min angle of rotation limit.
        """
        # split position into 4 bytes
        angle_loword = DXL_LOWORD(min_angle)
        angle_hiword = DXL_HIWORD(min_angle)

        response = self.write(
            servo_id, 
            DXL_MIN_POS_LIMIT, 
            ( DXL_LOBYTE(angle_loword), DXL_HIBYTE(angle_loword), DXL_LOBYTE(angle_hiword), DXL_HIBYTE(angle_hiword) )
        )
        if response:
            self.exception_on_error(
                response[8], servo_id, "setting min angle limit to %d" % angle_cw
            )
        return response

    def set_angle_limit_max(self, servo_id, max_angle):
        """
        Set the max angle of rotation limit.
        """
        # split position into 4 bytes
        angle_loword = DXL_LOWORD(max_angle)
        angle_hiword = DXL_HIWORD(max_angle)

        response = self.write(
            servo_id, 
            DXL_MAX_POS_LIMIT, 
            ( DXL_LOBYTE(angle_loword), DXL_HIBYTE(angle_loword), DXL_LOBYTE(angle_hiword), DXL_HIBYTE(angle_hiword) )
        )
        if response:
            self.exception_on_error(
                response[8], servo_id, "setting CCW angle limits to %d" % angle_ccw
            )
        return response

    def set_angle_limits(self, servo_id, min_angle, max_angle):
        """
        Set the min and max angle of rotation limits.
        """
        # split positions into 4 bytes
        min_angle_loword = DXL_LOWORD(min_angle)
        min_angle_hiword = DXL_HIWORD(min_angle)

        max_angle_loword = DXL_LOWORD(max_angle)
        max_angle_hiword = DXL_HIWORD(max_angle)

        # set 4 register values with low and high bytes for min and max angles
        response = self.write(
            servo_id, 
            DXL_MAX_POS_LIMIT, 
            ( DXL_LOBYTE(max_angle_loword), DXL_HIBYTE(max_angle_loword), DXL_LOBYTE(max_angle_hiword), DXL_HIBYTE(max_angle_hiword),
             DXL_LOBYTE(min_angle_loword), DXL_HIBYTE(min_angle_loword), DXL_LOBYTE(min_angle_hiword), DXL_HIBYTE(min_angle_hiword) )
        )
        if response:
            self.exception_on_error(
                response[8],
                servo_id,
                "setting CW and CCW angle limits to %d and %d" % (min_angle, max_angle),
            )
        return response

    def set_drive_mode(self, servo_id, is_slave=False, is_reverse=False):  # [TODO] Revise for DXL 2.0 intfc
        """
        Sets the drive mode
        """
        drive_mode = (is_slave << 1) + is_reverse

        response = self.write(servo_id, DXL_DRIVE_MODE, [drive_mode])
        if response:
            self.exception_on_error(
                response[8], servo_id, "setting drive mode to %d" % drive_mode
            )
        return response

    def set_voltage_limit_min(self, servo_id, min_voltage):
        """
        Set the minimum voltage limit.
        NOTE: the absolute min is 5v
        """

        if min_voltage < 5:
            min_voltage = 5
        minVal = int(min_voltage * 10)

        response = self.write(servo_id, DXL_MIN_VOLTAGE_LIMIT, [minVal])
        if response:
            self.exception_on_error(
                response[8],
                servo_id,
                "setting minimum voltage level to %d" % min_voltage,
            )
        return response

    def set_voltage_limit_max(self, servo_id, max_voltage):
        """
        Set the maximum voltage limit.
        NOTE: the absolute max is 20
        """

        if max_voltage > 20:
            max_voltage = 20
        maxVal = int(max_voltage * 10)

        response = self.write(servo_id, DXL_MAX_VOLTAGE_LIMIT, [maxVal])
        if response:
            self.exception_on_error(
                response[8],
                servo_id,
                "setting maximum voltage level to %d" % max_voltage,
            )
        return response

    def set_voltage_limits(self, servo_id, min_voltage, max_voltage):
        """
        Set the min and max voltage limits.
        NOTE: the absolute min is 5v and the absolute max is 25v
        """

        if min_voltage < 5:
            min_voltage = 5
        if max_voltage > 20:
            max_voltage = 20

        minVal = int(min_voltage * 10)
        maxVal = int(max_voltage * 10)

        response = self.write(servo_id, DXL_MIN_VOLTAGE_LIMIT, (minVal, maxVal))
        if response:
            self.exception_on_error(
                response[8],
                servo_id,
                "setting min and max voltage levels to %d and %d"
                % (min_voltage, max_voltage),
            )
        return response

    ###############################################################
    # These functions can send a single command to a single servo #
    ###############################################################

    def set_torque_enabled(self, servo_id, enabled):
        """
        Sets the value of the torque enabled register to 1 or 0. When the
        torque is disabled the servo can be moved manually while the motor is
        still powered.
        """
        response = self.write(servo_id, DXL_TORQUE_ENABLE, [enabled])
        if response:
            self.exception_on_error(
                response[8], servo_id, "%sabling torque" % "en" if enabled else "dis"
            )
        return response

    def set_d_gain(self, servo_id, d_gain):  # [TODO] Revise for DXL 2.0 intfc
        """
        Sets the value of derivative action of PID controller.
        Gain value is in range 0 to 254.
        """
        response = self.write(servo_id, DXL_D_GAIN, [d_gain])
        if response:
            self.exception_on_error(
                response[8],
                servo_id,
                "setting D gain value of PID controller to %d" % d_gain,
            )
        return response

    def set_i_gain(self, servo_id, i_gain):  # [TODO] Revise for DXL 2.0 intfc
        """
        Sets the value of integral action of PID controller.
        Gain value is in range 0 to 254.
        """
        response = self.write(servo_id, DXL_I_GAIN, [i_gain])
        if response:
            self.exception_on_error(
                response[8],
                servo_id,
                "setting I gain value of PID controller to %d" % i_gain,
            )
        return response

    def set_p_gain(self, servo_id, p_gain):  # [TODO] Revise for DXL 2.0 intfc
        """
        Sets the value of proportional action of PID controller.
        Gain value is in range 0 to 254.
        """
        response = self.write(servo_id, DXL_P_GAIN, [p_gain])
        if response:
            self.exception_on_error(
                response[8],
                servo_id,
                "setting P gain value of PID controller to %d" % p_gain,
            )
        return response

    def set_acceleration(self, servo_id, acceleration):  # [TODO] Revise for DXL 2.0 intfc
        """
        Sets the acceleration. The unit is 8.583 Degree / sec^2.
        0 - acceleration control disabled, 1-254 - valid range for acceleration.
        """

        model = self.get_model_number(servo_id)
        if not model in DXL_MODEL_TO_PARAMS:
            raise UnsupportedFeatureError(model, DXL_GOAL_ACCELERATION)

        if DXL_GOAL_ACCELERATION in DXL_MODEL_TO_PARAMS[model]["features"]:
            response = self.write(servo_id, DXL_GOAL_ACCELERATION, (acceleration,))
            if response:
                self.exception_on_error(
                    response[8], servo_id, "setting acceleration to %d" % acceleration
                )
            return response
        else:
            raise UnsupportedFeatureError(model, DXL_GOAL_ACCELERATION)

    def set_position(self, servo_id, position):
        """
        Set the servo with servo_id to the specified goal position.
        Position value must be positive.
        """
        # split position into 4 bytes
        pos_loword = DXL_LOWORD(position)
        pos_hiword = DXL_HIWORD(position)
            
        response = self.write(
            servo_id, 
            DXL_GOAL_POSITION, 
            ( DXL_LOBYTE(pos_loword), DXL_HIBYTE(pos_loword), DXL_LOBYTE(pos_hiword), DXL_HIBYTE(pos_hiword) )
        )
        if response:
            self.exception_on_error(
                response[8], servo_id, "setting goal position to %d" % position
            )
        return response

    def set_speed(self, servo_id, speed):
        """
        Set the servo with servo_id to the specified goal speed.
        Speed can be negative only if the dynamixel is in "freespin" mode.
        """
        # split speed into 4 bytes
        speed = abs(speed)    # positive according to motor spec, for profile velocity
        speed_loword = DXL_LOWORD(speed)
        speed_hiword = DXL_HIWORD(speed)
            
        # set two register values with low and high byte for the speed
        response = self.write(
            servo_id, 
            DXL_PROFILE_VELOCITY, 
            ( DXL_LOBYTE(speed_loword), DXL_HIBYTE(speed_loword), DXL_LOBYTE(speed_hiword), DXL_HIBYTE(speed_hiword) )
        )
        if response:
            self.exception_on_error(
                response[8], servo_id, "setting moving speed to %d" % speed
            )
        return response

    def set_torque_limit(self, servo_id, torque):  # [TODO] Revise for DXL 2.0 intfc
        """
        Sets the value of the maximum torque limit for servo with id servo_id.
        Valid values are 0 to 1023 (0x3FF), and the unit is about 0.1%.
        For example, if the value is 512 only 50% of the maximum torque will be used.
        If the power is turned on, the value of Max Torque (Address 14, 15) is used as the initial value.
        """
        loVal = int(torque % 256)
        hiVal = int(torque >> 8)

        response = self.write(servo_id, DXL_TORQUE_LIMIT_L, (loVal, hiVal))
        if response:
            self.exception_on_error(
                response[8], servo_id, "setting torque limit to %d" % torque
            )
        return response

    def set_goal_torque(self, servo_id, torque):  # [TODO] Revise for DXL 2.0 intfc
        """
        Set the servo to torque control mode (similar to wheel mode, but controlling the torque)
        Valid values are from -1023 to 1023.
        Anything outside this range or 'None' disables torque control.
        """

        model = self.get_model_number(servo_id)
        if not model in DXL_MODEL_TO_PARAMS:
            raise UnsupportedFeatureError(model, DXL_TORQUE_CONTROL_MODE)

        valid_torque = torque is not None and torque >= -1023 and torque <= 1023
        if torque is not None and torque < 0:
            torque = 1024 - torque

        if DXL_TORQUE_CONTROL_MODE in DXL_MODEL_TO_PARAMS[model]["features"]:
            if valid_torque:
                loVal = int(torque % 256)
                hiVal = int(torque >> 8)
                response = self.write(servo_id, DXL_GOAL_TORQUE_L, (loVal, hiVal))
                if response:
                    self.exception_on_error(
                        response[8], servo_id, "setting goal torque to %d" % torque
                    )
            response = self.write(
                servo_id, DXL_TORQUE_CONTROL_MODE, (int(valid_torque),)
            )
            if response:
                self.exception_on_error(response[8], servo_id, "enabling torque mode")
            return response
        else:
            raise UnsupportedFeatureError(model, DXL_TORQUE_CONTROL_MODE)

    def set_position_and_speed(self, servo_id, position, speed):
        """
        Set the servo with servo_id to specified position and speed.
        Speed can be negative only if the dynamixel is in "freespin" mode.
        """
        # split speed into 4 bytes
        speed = abs(speed)    # positive according to motor spec, for profile velocity
        speed_loword = DXL_LOWORD(speed)
        speed_hiword = DXL_HIWORD(speed)

        # split position into 4 bytes
        position_loword = DXL_LOWORD(position)
        position_hiword = DXL_HIWORD(position)

        response = self.write(
            servo_id,
            DXL_PROFILE_VELOCITY,
            ( DXL_LOBYTE(speed_loword), DXL_HIBYTE(speed_loword), DXL_LOBYTE(speed_hiword), DXL_HIBYTE(speed_hiword), 
                DXL_LOBYTE(position_loword), DXL_HIBYTE(position_loword), DXL_LOBYTE(position_hiword), DXL_HIBYTE(position_hiword) ),
        )
        if response:
            self.exception_on_error(
                response[8],
                servo_id,
                "setting goal position to %d and moving speed to %d"
                % (position, speed),
            )
        return response

    def set_led(self, servo_id, led_state):
        """
        Turn the LED of servo motor on/off.
        Possible boolean state values:
            True - turn the LED on,
            False - turn the LED off.
        """
        response = self.write(servo_id, DXL_LED, [led_state])
        if response:
            self.exception_on_error(
                response[8], servo_id, "setting a LED to %s" % led_state
            )
        return response

    #################################################################
    # These functions can send multiple commands to multiple servos #
    # These commands are used in ROS wrapper as they don't send a   #
    # response packet, ROS wrapper gets motor states at a set rate  #
    #################################################################

    def set_multi_torque_enabled(self, valueTuples):
        """
        Method to set multiple servos torque enabled.
        Should be called as such:
        set_multi_servos_to_torque_enabled( (id1, True), (id2, True), (id3, True) )
        """
        # use sync write to broadcast multi servo message
        self.sync_write(DXL_TORQUE_ENABLE, tuple(valueTuples))

    def set_multi_position(self, valueTuples):
        """
        Set different positions for multiple servos.
        Should be called as such:
        set_multi_position( ( (id1, position1), (id2, position2), (id3, position3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for vals in valueTuples:
            sid = vals[0]
            position = vals[1]
            
            # split position into 4 bytes
            pos_loword = DXL_LOWORD(position)
            pos_hiword = DXL_HIWORD(position)            
            writeableVals.append(
                ( sid, 
                DXL_LOBYTE(pos_loword), DXL_HIBYTE(pos_loword), DXL_LOBYTE(pos_hiword), DXL_HIBYTE(pos_hiword) )
            )

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_GOAL_POSITION, writeableVals)

    def set_multi_speed(self, valueTuples):  # [TODO] Change  name to set_multi_profile_velocity
        """
        Set different speeds for multiple servos.
        Should be called as such:
        set_multi_speed( ( (id1, speed1), (id2, speed2), (id3, speed3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for vals in valueTuples:
            sid = vals[0]
            speed = vals[1]

            # split speed into 4 bytes
            speed = abs(speed)    # positive according to motor spec, for profile velocity
            speed_loword = DXL_LOWORD(speed)
            speed_hiword = DXL_HIWORD(speed)
            
            writeableVals.append(
                ( sid, 
                DXL_LOBYTE(speed_loword), DXL_HIBYTE(speed_loword), DXL_LOBYTE(speed_hiword), DXL_HIBYTE(speed_hiword) )
            )
            
        # use sync write to broadcast multi servo message
        self.sync_write(DXL_PROFILE_VELOCITY, writeableVals)

    def set_multi_torque_limit(self, valueTuples):  # [TODO] Revise for DXL 2.0 intfc
        """
        Set different torque limits for multiple servos.
        Should be called as such:
        set_multi_torque_limit( ( (id1, torque1), (id2, torque2), (id3, torque3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for sid, torque in valueTuples:
            # split torque into 2 bytes
            loVal = int(torque % 256)
            hiVal = int(torque >> 8)
            writeableVals.append((sid, loVal, hiVal))

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_TORQUE_LIMIT_L, writeableVals)

    def set_multi_position_and_speed(self, valueTuples):
        """
        Set different positions and speeds for multiple servos.
        Should be called as such:
        set_multi_position_and_speed( ( (id1, position1, speed1), (id2, position2, speed2), (id3, position3, speed3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for vals in valueTuples:
            sid = vals[0]
            position = vals[1]
            speed = vals[2]

            # split speed into 4 bytes
            speed = abs(speed)    # positive according to motor spec, for profile velocity
            speed_loword = DXL_LOWORD(speed)
            speed_hiword = DXL_HIWORD(speed)

            # split position into 4 bytes
            position_loword = DXL_LOWORD(position)
            position_hiword = DXL_HIWORD(position)
            
            writeableVals.append(
                ( sid, 
                DXL_LOBYTE(speed_loword), DXL_HIBYTE(speed_loword), DXL_LOBYTE(speed_hiword), DXL_HIBYTE(speed_hiword), 
                DXL_LOBYTE(position_loword), DXL_HIBYTE(position_loword), DXL_LOBYTE(position_hiword), DXL_HIBYTE(position_hiword) )
            )

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_PROFILE_VELOCITY, tuple(writeableVals))

    #################################
    # Servo status access functions #
    #################################

    def get_model_number(self, servo_id):
        """Reads the servo's model number (e.g. 12 for AX-12+)."""
        response = self.read(servo_id, DXL_MODEL_NUMBER, 2)
        if response:
            self.exception_on_error(response[8], servo_id, "fetching model number")
        return DXL_MAKEWORD(response[9], response[10])

    def get_firmware_version(self, servo_id):
        """Reads the servo's firmware version."""
        response = self.read(servo_id, DXL_VERSION, 1)
        if response:
            self.exception_on_error(response[8], servo_id, "fetching firmware version")
        return response[9]

    def get_return_delay_time(self, servo_id):
        """Reads the servo's return delay time."""
        response = self.read(servo_id, DXL_RETURN_DELAY_TIME, 1)
        if response:
            self.exception_on_error(response[8], servo_id, "fetching return delay time")
        return response[9]

    def get_angle_limits(self, servo_id):
        """
        Returns the min and max angle limits from the specified servo.
        """
        # read in 8 consecutive bytes starting with low value of max angle limit
        response = self.read(servo_id, DXL_MAX_POS_LIMIT, 8)
        if response:
            self.exception_on_error(
                response[8], servo_id, "fetching max/min angle limits"
            )
        # extract data values from the raw data
        max_limit_lword = DXL_MAKEWORD(response[9], response[10])
        max_limit_hword = DXL_MAKEWORD(response[11], response[12])
        max_limit = DXL_MAKEDWORD(max_limit_lword, max_limit_hword)

        min_limit_lword = DXL_MAKEWORD(response[13], response[14])
        min_limit_hword = DXL_MAKEWORD(response[15], response[16])
        min_limit = DXL_MAKEDWORD(min_limit_lword, min_limit_hword)

        # return the data in a dictionary
        return {"min": min_limit, "max": max_limit}

    def get_drive_mode(self, servo_id):
        """Reads the servo's drive mode."""
        response = self.read(servo_id, DXL_DRIVE_MODE, 1)
        if response:
            self.exception_on_error(response[8], servo_id, "fetching drive mode")
        return response[9]

    def get_voltage_limits(self, servo_id):
        """
        Returns the min and max voltage limits from the specified servo.
        """
        response = self.read(servo_id, DXL_MAX_VOLTAGE_LIMIT, 4)
        if response:
            self.exception_on_error(response[8], servo_id, "fetching voltage limits")
        # extract data valus from the raw data
        max_voltage = DXL_MAKEWORD(response[9], response[10]) / 10.0
        min_voltage = DXL_MAKEWORD(response[11], response[12]) / 10.0

        # return the data in a dictionary
        return {"min": min_voltage, "max": max_voltage}

    def get_goal_position(self, servo_id):
        """Reads the servo's goal position value from its registers."""
        response = self.read(servo_id, DXL_GOAL_POSITION, 4)
        if response:
            self.exception_on_error(response[8], servo_id, "fetching goal position")
        goal_lword = DXL_MAKEWORD(response[9], response[10])
        goal_hword = DXL_MAKEWORD(response[11], response[12])
        goal_binary = DXL_MAKEDWORD(goal_lword, goal_hword)

        goal = DXL_DWORD_TO_INT32(goal_binary)

        return goal

    def get_position(self, servo_id):
        """Reads the servo's position value from its registers."""
        response = self.read(servo_id, DXL_PRESENT_POSITION, 4)
        if response:
            self.exception_on_error(response[8], servo_id, "fetching present position")
        position_lword = DXL_MAKEWORD(response[9], response[10])
        position_hword = DXL_MAKEWORD(response[11], response[12])
        position_binary = DXL_MAKEDWORD(position_lword, position_hword)

        position = DXL_DWORD_TO_INT32(position_binary)

        return position

    def get_speed(self, servo_id):
        """Reads the servo's speed value from its registers."""
        response = self.read(servo_id, DXL_PRESENT_VELOCITY, 4)
        if response:
            self.exception_on_error(response[8], servo_id, "fetching present velocity")
        speed_lword = DXL_MAKEWORD(response[9], response[10])
        speed_hword = DXL_MAKEWORD(response[11], response[12])
        speed_binary = DXL_MAKEDWORD(speed_lword, speed_hword)

        speed = DXL_DWORD_TO_INT32(speed_binary)
            
        return speed

    def get_voltage(self, servo_id):
        """Reads the servo's voltage."""
        response = self.read(servo_id, DXL_PRESENT_VOLTAGE, 2)
        if response:
            self.exception_on_error(response[8], servo_id, "fetching supplied voltage")
        return DXL_MAKEWORD(response[9], response[10]) / 10.0

    def get_current(self, servo_id):
        """Reads the servo's current consumption (if supported by model)"""
        model = self.get_model_number(servo_id)
        if not model in DXL_MODEL_TO_PARAMS:
            raise UnsupportedFeatureError(model, DXL_PRESENT_CURRENT)

        if DXL_PRESENT_CURRENT in DXL_MODEL_TO_PARAMS[model]["features"]:
            response = self.read(servo_id, DXL_PRESENT_CURRENT, 2)
            if response:
                self.exception_on_error(
                    response[8], servo_id, "fetching sensed current"
                )
            current_binary = DXL_MAKEWORD(response[5], response[6])
            current = DXL_WORD_TO_INT16(current_binary)
            return 0.00269 * current

        else:
            raise UnsupportedFeatureError(model, DXL_PRESENT_CURRENT)

    def get_feedback(self, servo_id):
        """
        Returns the id, goal, position, error, speed, current, voltage, temperature
        and moving values from the specified servo.
        """
        # read in 17 consecutive bytes starting with low value for goal position
        response = self.read(servo_id, DXL_GOAL_POSITION, 31)   # goal position -> present position

        if response:
            self.exception_on_error(response[8], servo_id, "fetching full servo status")
        if len(response) == 43:  # 31 (data) + 11 (header->len_h, crc_l, crc_h) + 1 (timestamp from read())
            # extract data values from the raw data
            goal_lword = DXL_MAKEWORD(response[9], response[10])
            goal_hword = DXL_MAKEWORD(response[11], response[12])
            goal_binary = DXL_MAKEDWORD(goal_lword, goal_hword)
            goal = DXL_DWORD_TO_INT32(goal_binary)

            position_lword = DXL_MAKEWORD(response[25], response[26])
            position_hword = DXL_MAKEWORD(response[27], response[28])
            position_binary = DXL_MAKEDWORD(position_lword, position_hword)
            position = DXL_DWORD_TO_INT32(position_binary)

            error = position - goal

            speed_lword = DXL_MAKEWORD(response[21], response[22])
            speed_hword = DXL_MAKEWORD(response[23], response[24])
            speed_binary = DXL_MAKEDWORD(speed_lword, speed_hword)
            speed = DXL_DWORD_TO_INT32(speed_binary)
              
            current_raw = DXL_MAKEWORD(response[19], response[20])
            current = 0.00269 * DXL_WORD_TO_INT16(current_raw)

            voltage = DXL_MAKEWORD(response[37], response[38]) / 10.0  # Volts
            
            temperature = response[39]    # deg C
            
            moving = response[15]
            
            moving_status_byte = response[16]     # moving status bit breakdown
            velocity_profile = (moving_status_byte & 0x30) >> 4
            following_error = moving_status_byte & 0x08
            profile_ongoing = moving_status_byte & 0x02
            in_position = moving_status_byte & 0x01
            moving_status = { "velocity_profile": velocity_profile, 
                              "following_error": bool(following_error), 
                              "profile_ongoing": bool(profile_ongoing), 
                              "in_position": bool(in_position),
                            }
            
            timestamp = time.time()

            # return the data in a dictionary
            return {
                "timestamp": timestamp,
                "id": servo_id,
                "goal": goal,
                "position": position,
                "error": error,
                "speed": speed,
                "current": current,
                "voltage": voltage,
                "temperature": temperature,
                "moving": bool(moving),
                "moving_status": moving_status,
            }

    def get_led(self, servo_id):
        """
        Get status of the LED. Boolean return values:
            True - LED is on,
            False - LED is off.
        """
        response = self.read(servo_id, DXL_LED, 1)
        if response:
            self.exception_on_error(response[8], servo_id, "fetching LED status")

        return bool(response[9])

    def exception_on_error(self, error_code, servo_id, command_failed):
        global exception
        exception = None
        ex_message = "[servo #%d on %s@%sbps]: %s failed" % (
            servo_id,
            self.ser.port,
            self.ser.baudrate,
            command_failed,
        )

        if not isinstance(error_code, int):
            msg = "Communication Error " + ex_message
            exception = NonfatalErrorCodeError(msg, 0)
            return
        if error_code == DXL_ACCESS_ERROR:
            msg = "Address Access Error " + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if error_code == DXL_DATA_LIMIT_ERROR:
            msg = "Data Limit Error " + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if error_code == DXL_DATA_LENGTH_ERROR:
            msg = "Data Required Length Error " + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if error_code == DXL_RANGE_ERROR:
            msg = "Data Allowable Range Error " + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if error_code == DXL_CHECKSUM_ERROR:
            msg = "Packet CRC Error " + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if error_code == DXL_INSTRUCTION_ERROR:
            msg = "Instruction Error " + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if error_code == DXL_RESULT_FAIL_ERROR:
            msg = "Result Failed Error " + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)


class SerialOpenError(Exception):
    def __init__(self, port, baud):
        Exception.__init__(self)
        self.message = "Cannot open port '%s' at %d bps" % (port, baud)
        self.port = port
        self.baud = baud

    def __str__(self):
        return self.message


class ChecksumError(Exception):
    def __init__(self, servo_id, response, checksum):
        Exception.__init__(self)
        self.message = (
            "Checksum received from motor %d does not match the expected one (%d != %d)"
            % (servo_id, response[-1], checksum)
        )
        self.response_data = response
        self.expected_checksum = checksum

    def __str__(self):
        return self.message


class FatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message


class NonfatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message


class ErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message


class DroppedPacketError(Exception):
    def __init__(self, message):
        Exception.__init__(self)
        self.message = message

    def __str__(self):
        return self.message


class UnsupportedFeatureError(Exception):
    def __init__(self, model_id, feature_id):
        Exception.__init__(self)
        if model_id in DXL_MODEL_TO_PARAMS:
            model = DXL_MODEL_TO_PARAMS[model_id]["name"]
        else:
            model = "Unknown"
        self.message = "Feature %d not supported by model %d (%s)" % (
            feature_id,
            model_id,
            model,
        )

    def __str__(self):
        return self.message
