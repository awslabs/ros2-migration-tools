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

""" A class and method for porting ROS1 c++ source code to ROS2 """
import re
import sys
from .constants import CPPSourceChanges
from .utils import find_warnings, get_functions_with

class CPPSourcePorter():
    """ Class containing static methods to change and print warning on C++ source code """
    @staticmethod
    def port(source, extra_rules=[], extra_warnings=[]):
        """
        Makes some automatic changes to ROS1 c++ source code to port to ROS2 and prints
        warnings for more complex cases.

        Arguments:
            source - A string of the c++ source code
            extra_rules - a list of functions to apply changes to the source code
            extra_warnings - a list of functions that return warnings about the source code
        Returns:
            The new source code
        """
        rules = get_functions_with(criteria=lambda name: name.startswith("rule"),
                                   from_class=CPPSourcePorter)

        for rule in rules + extra_rules:
            source = rule(source)

        warning_checks = get_functions_with(criteria=lambda name: name.startswith("warn"),
                                            from_class=CPPSourcePorter)

        warnings = []
        source_lines = source.split("\n")
        for warning_check in warning_checks + extra_warnings:
            warnings.extend(warning_check(source_lines))

        for warning in sorted(warnings, key=lambda warning: warning.line_number):
            print(str(warning))

        return source

    #########################
    #        RULES          #
    #########################

    @staticmethod
    def rule_replace_simple(source):

        for match, replacement in CPPSourceChanges.CPP_SOURCE_REPLACEMENTS.items():
            regex = re.compile(match, re.MULTILINE)
            source = regex.sub(replacement, source)

        return source

    #########################
    #      WARNINGS         #
    #########################

    @staticmethod
    def warn_ros_namespace(source_lines):
        warning_str = "ros namespace is only for ros1, use the rclcpp equivalent"
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "ros::" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_ros_time(source_lines):
        warning_str = "ros::Time is not supported, try rclcpp::Clock or builtin_interfaces::msg::Time"
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "ros::Time" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_msg_import(source_lines):
        warning_str = "Messages and Services namesapces now use a subnamespace (msg or srvcs)"
        #Importing a message but not using a msg subnamespace
        warning_condition = lambda line: "#include" in line and (("msg" in line) != ("/msg/" in line))
        return find_warnings(source_lines=source_lines,
                             warning_condition=warning_condition,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_msg_data(source_lines):
        warning_str = "This line may be accessing message data, messages are now pointers and should use ->"
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: ".data" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_ros_logging(source_lines):
        warning_str = ("ROS logging statements (e.g. ROS_INFO(<format_str>, <arguments>)) "
                       "have been replaced with RCLCPP logging statements (e.g. RCLCPP_INFO(<logger>, <format_str>, <arguments))")
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "ROS_" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_node_spin(source_lines):
        warning_str = "ros::spin has been replaced with rclcpp::spin and takes a node as an argument"
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "ros::spin" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_node_decl(source_lines):
        warning_str = "Nodes are now initialized differently, see documentation for details"
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "ros::NodeHandle" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_publish(source_lines):
        warning_str = "Publishers are now pointers"
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: ".publish" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_publish_creation(source_lines):
        warning_str = "Creation of publishers has changed, see documentation for details"
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: "ros::Publisher" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_subscribe(source_lines):
        warning_str = "Subscribers are now pointers"
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: ".subscribe" in line,
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_node_param(source_lines):
        warning_str = ("This line may be trying to access node parameters."
                       "In ROS2 node parameters are accessed using ->get_parameters"
                       "See documentation at: http://docs.ros2.org/beta1/api/rclcpp/classrclcpp_1_1node_1_1_node.html")
        return find_warnings(source_lines=source_lines,
                             warning_condition=lambda line: any(param in line for param in [".param", ".getParam", ".setParam", ".hasParam", ".deleteParam"]),
                             warning_msg=lambda line: warning_str)

    @staticmethod
    def warn_general_replacements(source_lines):
        warnings = []
        for condition, replacement_warning in CPPSourceChanges.REPLACEMENT_WARNINGS.items():
            warnings.extend(find_warnings(source_lines=source_lines,
                                          warning_condition=lambda line: bool(re.compile(condition).match(line)),
                                          warning_msg=lambda line: replacement_warning))
        return warnings

    @staticmethod
    def warn_boost_replacements(source_lines):
        warnings = []
        warning_str = ("C++11 has implemented much of the boost functionality, please use the std library equivalent if possible."
                       "%s has been replaced with %s")
        for boost, replace in CPPSourceChanges.BOOST_REPLACEMENTS.items():
            warnings.extend(find_warnings(source_lines=source_lines,
                                          warning_condition=lambda line: bool(re.compile(boost).match(line)),
                                          warning_msg=lambda line: warning_str % (boost, replace)))
        return warnings


def port_cpp(src, dst):
    with open(src, 'r') as src_file:
        source = src_file.read()
    with open(dst, 'w') as dst_file:
        dst_file.write(CPPSourcePorter.port(source=source))

def main():
    with open(sys.argv[1], 'r') as file:
        print(CPPSourcePorter.port(source=file.read()))


if __name__ == '__main__':
    main()
