#!/usr/bin/env python
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


import sys
from functools import partial
from optparse import OptionParser

import rospy

from dynamixel_controllers.srv import StartController
from dynamixel_controllers.srv import StopController
from dynamixel_controllers.srv import RestartController


def get_controller_params(controller_name):
    try:
        controller = rospy.get_param(controller_name + "/controller")
        return [controller[key] for key in ["package", "module", "type"]]
    except KeyError as e:
        rospy.logerr(
            "[%s] configuration error: could not find controller parameters on parameter server"
            % controller_name
        )
        sys.exit(1)
    except Exception as e:
        rospy.logerr("[%s]: %s" % (controller_name, e))
        sys.exit(1)


def manage_controller(
    controller_name,
    port_namespace,
    command,
    deps,
    start,
    stop,
    restart,
):
    package_path, module_name, class_name = get_controller_params(controller_name)
    lookup = {
        "start": partial(
            start,
            port_namespace,
            package_path,
            module_name,
            class_name,
            controller_name,
            deps,
        ),
        "stop": partial(stop, controller_name),
        "restart": partial(
            restart,
            port_namespace,
            package_path,
            module_name,
            class_name,
            controller_name,
            deps,
        ),
    }
    func = lookup.get(command.lower())
    if not func:
        rospy.logerr("Invalid command.")
        parser = get_parser()
        parser.print_help()

    try:
        response = func()
        if response.success:
            rospy.loginfo(response.reason)
        else:
            rospy.logerr(response.reason)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def get_parser():
    parser = OptionParser()
    parser.add_option(
        "-m",
        "--manager",
        metavar="MANAGER",
        help="specified serial port is managed by MANAGER",
    )
    parser.add_option(
        "-p",
        "--port",
        metavar="PORT",
        help="motors of specified controllers are connected to PORT",
    )
    parser.add_option(
        "-t",
        "--type",
        metavar="TYPE",
        default="simple",
        choices=("simple", "meta"),
        help="type of controller to be loaded (simple|meta) [default: %default]",
    )
    parser.add_option(
        "-c",
        "--command",
        metavar="COMMAND",
        default="start",
        choices=("start", "stop", "restart"),
        help="command to perform on specified controllers: start, stop or restart [default: %default]",
    )
    return parser


def main(argv):
    rospy.init_node("controller_spawner", anonymous=True)

    parser = get_parser()
    options, args = parser.parse_args(argv[1:])

    if not args:
        parser.error("specify at least one controller name")

    controller_type = options.type
    manager_namespace = options.manager
    port_namespace = "meta" if controller_type == "meta" else options.port
    command = options.command
    joint_controllers = args

    parent_namespace = (
        "global" if rospy.get_namespace() == "/" else rospy.get_namespace()
    )
    rospy.loginfo(
        "%s controller_spawner: waiting for controller_manager %s to startup in %s namespace..."
        % (port_namespace, manager_namespace, parent_namespace)
    )

    controllers = []
    for name, spec in [
        ("start_controller", StartController),
        ("stop_controller", StopController),
        ("restart_controller", RestartController),
    ]:
        service_name = "{}/{}/{}".format(manager_namespace, port_namespace, name)
        rospy.wait_for_service(service_name)
        controller = rospy.ServiceProxy(service_name, spec)
        controllers.append(controller)

    rospy.loginfo(
        "%s controller_spawner: All services are up, spawning controllers..."
        % port_namespace
    )

    if controller_type == "simple":
        for controller_name in joint_controllers:
            manage_controller(
                controller_name,
                port_namespace,
                command,
                [],
                controllers[0],
                controllers[1],
                controllers[2],
            )
    elif controller_type == "meta":
        controller_name = joint_controllers[0]
        dependencies = joint_controllers[1:]
        manage_controller(
            controller_name,
            port_namespace,
            command,
            dependencies,
            controllers[0],
            controllers[1],
            controllers[2],
        )


if __name__ == "__main__":
    try:
        main(rospy.myargv())
    except rospy.ROSInterruptException:
        pass
