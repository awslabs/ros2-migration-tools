# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#  
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#  
#      http://www.apache.org/licenses/LICENSE-2.0
#  
# or in the "license" file accompanying this file. This file is distributed 
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either 
# express or implied. See the License for the specific language governing 
# permissions and limitations under the License.

""" Contains a class and method for porting ROS1 python code to ROS2 """

import sys
from .utils import find_warnings, get_functions_with

class PythonSourcePorter():
    """
    Contains rules for automatically changing ros1 python code to ros2
    and warnings for elements that may need to be changed but aren't be done
    automatically.
    """
    @staticmethod
    def port(source, extra_rules=[], extra_warnings=[]):
        """
        Makes some automatic changes to ROS1 python source code to port to ROS2 and prints
        warnings for more complex cases.

        Arguments:
            source - A string of the python source code
            extra_rules - a list of functions to apply changes to the source code
            extra_warnings - a list of functions that return warnings about the source code
        Returns:
            The new source code
        """
        #Pulls out all methods in this class with name starting with "rule"
        rules = get_functions_with(criteria=lambda name: name.startswith("rule"),
                                   from_class=PythonSourcePorter)

        for rule in rules + extra_rules:
            source = rule(source)

        #Pulls out all methods in this class with name starting with "warn"
        warning_checks = get_functions_with(criteria=lambda name: name.startswith("warn"),
                                            from_class=PythonSourcePorter)

        warnings = []
        source_lines = source.split("\n")
        for warning_check in warning_checks + extra_warnings:
            warnings.extend(warning_check(source_lines))

        for warning in sorted(warnings, key=lambda warning: warning.line_number):
            print(str(warning))

        return source

    #########################
    #      WARNINGS         #
    #########################

    @staticmethod
    def warn_rospy(source_lines):
        warning_str = "rospy is the ros1 python library, use the rclpy equivalent"
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "rospy" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_publisher(source_lines):
        warning_str = ("Publishers are created with the <node>.create_publisher method.\n"
                       "See: https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/node.py")
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "rospy.Publisher" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_subscriber(source_lines):
        warning_str = ("Subscribers are created with the <node>.create_subscription method.\n"
                       "See: https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/node.py")
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "rospy.Subscriber" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_logging(source_lines):
        warning_str = "rospy.log<sev> has been replaced with <node>.get_logger().<sev>"
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "rospy.log" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_time(source_lines):
        warning_str = ("rospy.time has been replaced, try rclpy.clock. Not all features currently exist.\n"
                       "See: https://github.com/ros2/rclpy/issues/186")
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "rospy.time" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_node_creation(source_lines):
        warning_str = ("In ROS1 init_node initialized ros and created a node for that process.\n"
                       "In ROS2 you need to initialize ros and create the node separately"
                       "rclp.y.init(args)"
                       "node = rclpy.create_node(<name>)")
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "rospy.init_node" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_shutdown(source_lines):
        warning_str = ("is_shutdown isn't implemented in ros2 yet\n"
                       "See:  https://github.com/ros2/rclpy/issues/190")
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "rospy.is_shutdown" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_exceptions(source_lines):
        warning_str = ("Some exceptions have changed names and some have not been implemented in ROS2\n"
                       "See  https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/exceptions.py for existing exceptions")
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "Exception" in line,
                             warning_msg=lambda line: warning_str)



def port_python(src, dst):
    """
    Makes some automatic changes ROS1 python source code to ROS2 and prints warnings
    Arguments:
        src - the source file for the ROS1 python code
        dst - the destination file for ROS2 python code
    """
    with open(src, 'r') as src_file:
        source = src_file.read()
    with open(dst, 'w') as dst_file:
        dst_file.write(PythonSourcePorter.port(source=source))

def main():
    with open(sys.argv[1], 'r') as file:
        print(PythonSourcePorter.port(source=file.read()))

if __name__ == '__main__':
    main()
