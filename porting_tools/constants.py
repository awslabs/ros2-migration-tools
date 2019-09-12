# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

PACKAGE_XML_ELEMENT_ORDER = [
    "name",
    "version",
    "description",
    "author",
    "maintainer",
    "license",
    "url",
    "buildtool_depend",
    "build_depend",
    "build_export_depend",
    "depend",
    "exec_depend",
    "test_depend",
    "member_of_group",
    "doc_depend",
    "conflict",
    "replace",
    "export",
]

class CatkinToAmentMigration:
    DEFAULT_SRC_PACKAGE_XML = "package.xml"

    DEFAULT_SRC_CMAKE_LISTS = "CMakeLists.txt"

    DEFAULT_DST_PACKAGE_XML = "ros2_package.xml"

    DEFAULT_DST_CMAKE_LISTS= "ros2_CMakeLists.txt"

    CMAKE_LISTS_RENAMED_VARIABLES = {
        "${CATKIN_DEVEL_PREFIX}/": "",
        "${CATKIN_GLOBAL_BIN_DESTINATION}": "bin",
        "${CATKIN_GLOBAL_INCLUDE_DESTINATION}": "include",
        "${CATKIN_GLOBAL_LIB_DESTINATION}": "lib",
        "${CATKIN_GLOBAL_LIBEXEC_DESTINATION}": "lib",
        "${CATKIN_GLOBAL_SHARE_DESTINATION}": "share",
        "${CATKIN_PACKAGE_BIN_DESTINATION}": "bin",
        "${CATKIN_PACKAGE_INCLUDE_DESTINATION}": "include/${PROJECT_NAME}",
        "${CATKIN_PACKAGE_LIB_DESTINATION}": "lib",
        "${CATKIN_PACKAGE_SHARE_DESTINATION}": "share/${PROJECT_NAME}",
    }

    #Commands that are not used by ROS2
    CMAKE_LISTS_DELETED_COMMANDS = [
        "catkin_dstinations",
        "catkin_metapackage",
        "catkin_python_setup",
        "catkin_package",
        "generate_messages",
    ]

    #Commands that have the same semantics in ROS1 and ROS2
    CMAKE_LISTS_REPLACED_COMMANDS = {
        "catkin_add_gtest":"ament_add_gtest",
    }

    CLIENT_CONVERSION = {
        "roscpp":"rclcpp",
        "rospy":"rclpy",
    }


class ROS1ROS2Packages:
    # List of ROS packages that do not currently exist in ROS 2
    UNKNOWN_ROS_PACKAGES = [
        "dynamic_reconfigure",
        "roslib",
    ]

    # List of ROS packages that have been renamed in ROS 2
    RENAMED_ROS_PACKAGES = {
        "tf": "tf2",
        "roscpp": "rclcpp",
        "rospy": "rclpy",
        "nodelet": "rclcpp",  # nodelets have become components in ROS 2
        "message_generation": "rosidl_default_generators",
        "message_runtime": "rosidl_default_runtime",
        "rosconsole": "ros2_console",  # Until ROS 2 replacement exists
    }

    # List of packages that do not need to be found by CMake
    NO_CMAKE_FIND_PACKAGES = [
        "rosidl_default_runtime",
    ]





