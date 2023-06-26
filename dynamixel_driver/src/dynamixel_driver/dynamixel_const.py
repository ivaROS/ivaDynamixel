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


"""
Dynamixel Constants (DXL Protocol 2.0)
DYNAMIXEL Protocol 2.0 supported devices: 
  MX-28, MX-64, MX-106, X Series (2X Series included), PRO Series, P Series.
"""

# Control Table Constants
#   === EEPROM AREA === 
DXL_MODEL_NUMBER = 0
DXL_MODEL_INFO = 2
DXL_VERSION = 6
DXL_ID = 7
DXL_BAUD_RATE = 8
DXL_RETURN_DELAY_TIME = 9

DXL_DRIVE_MODE = 10
DXL_OPERATING_MODE = 11

DXL_SEC_ID = 12
DXL_PROTOCOL_TYPE = 13
DXL_HOMING_OFFSET = 20
DXL_MOVING_THRESHOLD = 24
DXL_TEMPERATURE_LIMIT = 31

DXL_MAX_VOLTAGE_LIMIT = 32
DXL_MIN_VOLTAGE_LIMIT = 34

DXL_PWM_LIMIT = 36
DXL_CURRENT_LIMIT = 38
DXL_ACCELERATION_LIMIT = 40
DXL_VELOCITY_LIMIT = 44

DXL_MAX_POS_LIMIT = 48
DXL_MIN_POS_LIMIT = 52

DXL_STARTUP_CONFIG = 60
DXL_SHUTDOWN = 63

#   === RAM AREA === 
DXL_TORQUE_ENABLE = 64
DXL_LED = 65

DXL_STATUS_RETURN_LEVEL = 68
DXL_REGISTERED_INSTRUCTION = 69
DXL_HW_ERROR_STATUS = 70

DXL_VELOCITY_I_GAIN = 76
DXL_VELOCITY_P_GAIN = 78

DXL_POSITION_D_GAIN = 80
DXL_POSITION_I_GAIN = 82
DXL_POSITION_P_GAIN = 84

DXL_FF_2ND_GAIN = 88
DXL_FF_1ST_GAIN = 90

DXL_BUS_WATCHDOG = 98

DXL_GOAL_PWM = 100
DXL_GOAL_CURRENT = 102
DXL_GOAL_VELOCITY = 104

DXL_PROFILE_ACCELERATION = 108
DXL_PROFILE_VELOCITY = 112

DXL_GOAL_POSITION = 116

DXL_REALTIME_TICK = 120

DXL_MOVING = 122
DXL_MOVING_STATUS = 123

DXL_PRESENT_PWM = 124
DXL_PRESENT_LOAD = 126        # for MX-28(2.0) and some X-series motors
DXL_PRESENT_CURRENT = 126     # for other motor modele, e.g. MX-64(2.0), MX-106(2.0), XM430-W350
DXL_PRESENT_VELOCITY = 128
DXL_PRESENT_POSITION = 132

DXL_VELOCITY_TRAJECTORY = 136
DXL_POSITION_TRAJECTORY = 140

DXL_PRESENT_VOLTAGE = 144
DXL_PRESENT_TEMPERATURE = 146

DXL_BACKUP_READY = 147


# Status Return Levels
DXL_RETURN_NONE = 0
DXL_RETURN_READ = 1
DXL_RETURN_ALL = 2

# Instruction Set
DXL_PING = 1
DXL_READ_DATA = 2
DXL_WRITE_DATA = 3
DXL_REG_WRITE = 4
DXL_ACTION = 5
DXL_RESET = 6
DXL_REBOOT = 8
DXL_CLEAR = 16
DXL_STATUS_RETURN = 85
DXL_SYNC_WRITE = 131
#   [NOTE] Currently unsupported instructions: 
#     Control Table Back-up (0x20)
#     Sync Read (0x82)
#     Fast Sync Read (0x8A)
#     Bulk Read (0x92)
#     Bulk Write (0x93)
#     Fast Bulk Read (0x9A)

# Broadcast Constant
DXL_BROADCAST = 254

# Error Codes
DXL_ACCESS_ERROR = 7
DXL_DATA_LIMIT_ERROR = 6
DXL_DATA_LENGTH_ERROR = 5
DXL_RANGE_ERROR = 4
DXL_CHECKSUM_ERROR = 3
DXL_INSTRUCTION_ERROR = 2
DXL_RESULT_FAIL_ERROR = 1
DXL_NO_ERROR = 0

# Static parameters
DXL_MIN_POS_P_GAIN = 0
DXL_MAX_POS_P_GAIN = 16383

DXL_MIN_POS_I_GAIN = 0
DXL_MAX_POS_I_GAIN = 16383

DXL_MIN_POS_D_GAIN = 0
DXL_MAX_POS_D_GAIN = 16383

DXL_MIN_VEL_P_GAIN = 0
DXL_MAX_VEL_P_GAIN = 16383

DXL_MIN_VEL_I_GAIN = 0
DXL_MAX_VEL_I_GAIN = 16383

DXL_MIN_FF_1ST_GAIN = 0
DXL_MAX_FF_1ST_GAIN = 16383

DXL_MIN_FF_2ND_GAIN = 0
DXL_MAX_FF_2ND_GAIN = 16383


KGCM_TO_NM = 0.0980665  # 1 kg-cm is that many N-m
RPM_TO_RADSEC = 0.104719755  # 1 RPM is that many rad/sec

# maximum holding torque is in N-m per volt
# maximum velocity is in rad/sec per volt
DXL_MODEL_TO_PARAMS = {
    30: {
        "name": "MX-28(2.0)",
        "encoder_resolution": 4096,
        "range_degrees": 360.0,
        "torque_per_volt": 2.5 / 12.0,  #  2.5 NM @ 12V
        "velocity_per_volt": (55 * RPM_TO_RADSEC) / 12.0,  #  55 RPM @ 12.0V
        "rpm_per_tick": 0.229,
        "rpm_sq_per_tick": 214.577,       # [TODO] not implemented
        "load_per_tick": 0.1,             # % / tick    # [TODO] not implemented
        "pwm_per_tick": 0.113,            # % / tick 
        "max_velocity_tick": 1023,        # max configurable velocity (units: ticks)
        "max_load_tick": 1000,            # max configurable current (units: ticks, -1000 to +1000)    # [TODO] not implemented
        "max_pwm_tick": 885,              # max configurable PWM (units: ticks)
        "max_acceleration_tick": 32767,   # max configurable acceleration (units: ticks)    # [TODO] not implemented
        "features": [DXL_ACCELERATION_LIMIT, DXL_PRESENT_LOAD],
    },
    311: {
        "name": "MX-64(2.0)",
        "encoder_resolution": 4096,
        "range_degrees": 360.0,
        "torque_per_volt": 6.0 / 12.0,  #  6 NM @ 12V
        "velocity_per_volt": (63 * RPM_TO_RADSEC) / 12.0,  #  63 RPM @ 12.0V
        "rpm_per_tick": 0.229,
        "rpm_sq_per_tick": 214.577,       # [TODO] not implemented
        "current_per_tick": 0.00336,      # mA / tick
        "pwm_per_tick": 0.113,            # % / tick 
        "max_velocity_tick": 1023,        # max configurable velocity (units: ticks)
        "max_current_tick": 1941,         # max configurable current (units: ticks)
        "max_pwm_tick": 885,              # max configurable PWM (units: ticks)
        "max_acceleration_tick": 32767,   # max configurable acceleration (units: ticks)    # [TODO] not implemented
        "features": [DXL_ACCELERATION_LIMIT, DXL_PRESENT_CURRENT],
    },
    321: {
        "name": "MX-106(2.0)",
        "encoder_resolution": 4096,
        "range_degrees": 360.0,
        "torque_per_volt": 8.4 / 12.0,  #  8.4 NM @ 12V
        "velocity_per_volt": (45 * RPM_TO_RADSEC) / 12.0,  #  45 RPM @ 12.0V
        "rpm_per_tick": 0.229,
        "rpm_sq_per_tick": 214.577,       # [TODO] not implemented
        "current_per_tick": 0.00336,      # mA / tick
        "pwm_per_tick": 0.113,            # % / tick 
        "max_velocity_tick": 1023,        # max configurable velocity (units: ticks)
        "max_current_tick": 2047,         # max configurable current (units: ticks)
        "max_pwm_tick": 885,              # max configurable PWM (units: ticks)
        "max_acceleration_tick": 32767,   # max configurable acceleration (units: ticks)    # [TODO] not implemented
        "features": [DXL_ACCELERATION_LIMIT, DXL_PRESENT_CURRENT],
    },
    1020: {
        "name": "XM430-W350",
        "encoder_resolution": 4096,
        "range_degrees": 360.0,
        "torque_per_volt": 4.1 / 12.0,  #  8.4 NM @ 12V
        "velocity_per_volt": (46 * RPM_TO_RADSEC) / 12.0,  #  45 RPM @ 12.0V
        "rpm_per_tick": 0.229, 
        "current_per_tick": 0.00269,  # mA / tick
        "pwm_per_tick": 0.113,        # % / tick 
        "max_velocity_tick": 1023,    # max configurable velocity (units: ticks)
        "max_current_tick": 1193,     # max configurable current (units: ticks)
        "max_pwm_tick": 885,          # max configurable PWM (units: ticks)
        "features": [DXL_PRESENT_CURRENT],
    },
}
