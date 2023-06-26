# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
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


__author__ = "Antons Rebguns"
__copyright__ = "Copyright (c) 2010-2011 Antons Rebguns"

__license__ = "BSD"
__maintainer__ = "Antons Rebguns"
__email__ = "anton@email.arizona.edu"


import math

import rospy

from dynamixel_driver.dynamixel_const import *

from dynamixel_controllers.srv import SetSpeed
from dynamixel_controllers.srv import TorqueEnable
from dynamixel_controllers.srv import SetGain
from dynamixel_controllers.srv import SetTorqueLimit

from std_msgs.msg import Float64
from dynamixel_msgs.msg import MotorStateList
from dynamixel_msgs.msg import JointState


class JointController:
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        self.joint_name = rospy.get_param(self.controller_namespace + "/joint_name")
        self.joint_speed = rospy.get_param(
            self.controller_namespace + "/joint_speed", 1.0
        )
        
        self.pos_p_gain = rospy.get_param(
            self.controller_namespace + "/joint_pos_p_gain", None
        )
        self.pos_i_gain = rospy.get_param(
            self.controller_namespace + "/joint_pos_i_gain", None
        )
        self.pos_d_gain = rospy.get_param(
            self.controller_namespace + "/joint_pos_d_gain", None
        )

        self.vel_p_gain = rospy.get_param(
            self.controller_namespace + "/joint_vel_p_gain", None
        )
        self.vel_i_gain = rospy.get_param(
            self.controller_namespace + "/joint_vel_i_gain", None
        )

        self.ff_1st_gain = rospy.get_param(
            self.controller_namespace + "/joint_ff_1st_gain", None
        )
        self.ff_2nd_gain = rospy.get_param(
            self.controller_namespace + "/joint_ff_2nd_gain", None
        )

        self.torque_limit = rospy.get_param(
            self.controller_namespace + "/joint_torque_limit", None
        )

        self.__ensure_limits()

        self.speed_service = rospy.Service(
            self.controller_namespace + "/set_speed", SetSpeed, self.process_set_speed
        )
        self.torque_service = rospy.Service(
            self.controller_namespace + "/torque_enable",
            TorqueEnable,
            self.process_torque_enable,
        )
        self.torque_limit_service = rospy.Service(
            self.controller_namespace + "/set_torque_limit",
            SetTorqueLimit,
            self.process_set_torque_limit,
        )
        
        self.position_p_gain_service = rospy.Service(
            self.controller_namespace + "/position_p_gain",
            SetGain,
            self.process_set_pos_p_gain,
        )
        self.position_i_gain_service = rospy.Service(
            self.controller_namespace + "/position_i_gain",
            SetGain,
            self.process_set_pos_i_gain,
        )
        self.position_d_gain_service = rospy.Service(
            self.controller_namespace + "/position_d_gain",
            SetGain,
            self.process_set_pos_d_gain,
        )
        self.velocity_p_gain_service = rospy.Service(
            self.controller_namespace + "/velocity_p_gain",
            SetGain,
            self.process_set_vel_p_gain,
        )
        self.velocity_i_gain_service = rospy.Service(
            self.controller_namespace + "/velocity_i_gain",
            SetGain,
            self.process_set_vel_i_gain,
        )
        self.feedforward_1st_gain_service = rospy.Service(
            self.controller_namespace + "/feedforward_1st_gain",
            SetGain,
            self.process_set_ff_1st_gain,
        )
        self.feedforward_2nd_gain_service = rospy.Service(
            self.controller_namespace + "/feedforward_2nd_gain",
            SetGain,
            self.process_set_ff_2nd_gain,
        )

    def __ensure_limits(self):
        if self.pos_p_gain is not None:
            if self.pos_p_gain < DXL_MIN_POS_P_GAIN:
                self.pos_p_gain = DXL_MIN_POS_P_GAIN
            elif self.pos_p_gain > DXL_MAX_POS_P_GAIN:
                self.pos_p_gain = DXL_MAX_POS_P_GAIN
            else:
                self.pos_p_gain = int(self.pos_p_gain)

        if self.pos_i_gain is not None:
            if self.pos_i_gain < DXL_MIN_POS_I_GAIN:
                self.pos_i_gain = DXL_MIN_POS_I_GAIN
            elif self.pos_i_gain > DXL_MAX_POS_I_GAIN:
                self.pos_i_gain = DXL_MAX_POS_I_GAIN
            else:
                self.pos_i_gain = int(self.pos_i_gain)

        if self.pos_d_gain is not None:
            if self.pos_d_gain < DXL_MIN_POS_D_GAIN:
                self.pos_d_gain = DXL_MIN_POS_D_GAIN
            elif self.pos_d_gain > DXL_MAX_POS_D_GAIN:
                self.pos_d_gain = DXL_MAX_POS_D_GAIN
            else:
                self.pos_d_gain = int(self.pos_d_gain)

        if self.vel_p_gain is not None:
            if self.vel_p_gain < DXL_MIN_VEL_P_GAIN:
                self.vel_p_gain = DXL_MIN_VEL_P_GAIN
            elif self.vel_p_gain > DXL_MAX_VEL_P_GAIN:
                self.vel_p_gain = DXL_MAX_VEL_P_GAIN
            else:
                self.vel_p_gain = int(self.vel_p_gain)

        if self.vel_i_gain is not None:
            if self.vel_i_gain < DXL_MIN_VEL_I_GAIN:
                self.vel_i_gain = DXL_MIN_VEL_I_GAIN
            elif self.vel_i_gain > DXL_MAX_VEL_I_GAIN:
                self.vel_i_gain = DXL_MAX_VEL_I_GAIN
            else:
                self.vel_i_gain = int(self.vel_i_gain)

        if self.ff_1st_gain is not None:
            if self.ff_1st_gain < DXL_MIN_FF_1ST_GAIN:
                self.ff_1st_gain = DXL_MIN_FF_1ST_GAIN
            elif self.ff_1st_gain > DXL_MAX_FF_1ST_GAIN:
                self.ff_1st_gain = DXL_MAX_FF_1ST_GAIN
            else:
                self.ff_1st_gain = int(self.ff_1st_gain)

        if self.ff_2nd_gain is not None:
            if self.ff_2nd_gain < DXL_MIN_FF_2ND_GAIN:
                self.ff_2nd_gain = DXL_MIN_FF_2ND_GAIN
            elif self.ff_2nd_gain > DXL_MAX_FF_2ND_GAIN:
                self.ff_2nd_gain = DXL_MAX_FF_2ND_GAIN
            else:
                self.ff_2nd_gain = int(self.ff_2nd_gain)

        if self.torque_limit is not None:     # [TODO] change to current limit, add other limits; add model-based conversion factors
            if self.torque_limit < 0:
                self.torque_limit = 0.0
            elif self.torque_limit > 1:
                self.torque_limit = 1.0

    def initialize(self):
        raise NotImplementedError

    def start(self):
        self.running = True
        self.joint_state_pub = rospy.Publisher(
            self.controller_namespace + "/state", JointState, queue_size=1
        )
        self.command_sub = rospy.Subscriber(
            self.controller_namespace + "/command", Float64, self.process_command
        )
        self.motor_states_sub = rospy.Subscriber(
            "motor_states/%s" % self.port_namespace,
            MotorStateList,
            self.process_motor_states,
        )

    def stop(self):
        self.running = False
        self.joint_state_pub.unregister()
        self.motor_states_sub.unregister()
        self.command_sub.unregister()
        self.speed_service.shutdown("normal shutdown")
        self.torque_service.shutdown("normal shutdown")

    def set_torque_enable(self, torque_enable):
        raise NotImplementedError

    def set_speed(self, speed):
        raise NotImplementedError

    def set_pos_p_gain(self, gain):
        raise NotImplementedError

    def set_pos_i_gain(self, gain):
        raise NotImplementedError

    def set_pos_d_gain(self, gain):
        raise NotImplementedError

    def set_vel_p_gain(self, gain):
        raise NotImplementedError

    def set_vel_i_gain(self, gain):
        raise NotImplementedError

    def set_ff_1st_gain(self, gain):
        raise NotImplementedError

    def set_ff_2nd_gain(self, gain):
        raise NotImplementedError

    def set_torque_limit(self, max_torque):
        raise NotImplementedError

    def process_set_speed(self, req):
        self.set_speed(req.speed)
        return []  # success

    def process_torque_enable(self, req):
        self.set_torque_enable(req.torque_enable)
        return []

    def process_set_pos_p_gain(self, req):
        self.set_pos_p_gain(req.gain)
        return []

    def process_set_pos_i_gain(self, req):
        self.set_pos_i_gain(req.gain)
        return []

    def process_set_pos_d_gain(self, req):
        self.set_pos_d_gain(req.gain)
        return []

    def process_set_vel_p_gain(self, req):
        self.set_vel_p_gain(req.gain)
        return []

    def process_set_vel_i_gain(self, req):
        self.set_vel_i_gain(req.gain)
        return []

    def process_set_ff_1st_gain(self, req):
        self.set_ff_1st_gain(req.gain)
        return []

    def process_set_ff_2nd_gain(self, req):
        self.set_ff_2nd_gain(req.gain)
        return []

    def process_set_torque_limit(self, req):    # [TODO] re-implement as current limit
        self.set_torque_limit(req.torque_limit)
        return []

    def process_motor_states(self, state_list):
        raise NotImplementedError

    def process_command(self, msg):
        raise NotImplementedError

    def rad_to_raw(
        self, angle, initial_position_raw, flipped, encoder_ticks_per_radian
    ):
        """angle is in radians"""
        # print 'flipped = %s, angle_in = %f, init_raw = %d' % (str(flipped), angle, initial_position_raw)
        angle_raw = angle * encoder_ticks_per_radian
        # print 'angle = %f, val = %d' % (math.degrees(angle), int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw)))
        return int(
            round(
                initial_position_raw - angle_raw
                if flipped
                else initial_position_raw + angle_raw
            )
        )

    def raw_to_rad(self, raw, initial_position_raw, flipped, radians_per_encoder_tick):
        return (
            initial_position_raw - raw if flipped else raw - initial_position_raw
        ) * radians_per_encoder_tick
