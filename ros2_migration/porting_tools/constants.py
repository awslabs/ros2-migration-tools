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



class CPPSourceChanges():
    #Regexs for replacement strings between ROS1 and ROS2
    CPP_SOURCE_REPLACEMENTS = {
        r"ros::Rate" : r"rclcpp::Rate",
        r"ros::ok" : r"rclcpp::ok",
        r"ros/ros.h" : r"rclcpp/rclcpp.hpp",
        #Msgs and srvcs are now stored in a separate folder
        r"std_msgs/(?!msg/)(.*).h" : r"std_msgs/msg/\1.hpp",
        r"std_msgs::(?!msg::)(\S*)" : r"std_msgs::msg::\1",
        #String.h is now string.hpp
        r"std_msgs/(.*)String(.*)" : r"std_msgs/\1string\2",
        #Replace the node creation e.g.
        #ROS1:
        #   ros::init(argc, argv, "talker");
        #   ros::NodeHandle n;
        #ROS2:
        #   rclcpp::init(argc, argv);
        #   auto node = rclcpp::Node::make_shared("talker");
        #Ignores cases where multiple nodes are declared on one line
        r"ros::init\((.*),(.*),\s*([\S]*)\);([\s\S]*)ros::NodeHandle ([^,\(]*);" : r"rclcpp.init(\1,\2);\4auto \5 = rclcpp::Node::make_shared(\3);",
        # Replace standard message creation
        #ROS1:
        #  std_msgs::String msg;
        #ROS2:
        #  auto msg = std::make_shared<std_msgs::msg::String>();
        #Also works if std_msgs has already been replaced with std_msgs::msgs
        r"^([\s]*)std_msgs::(msg::)?([\S]*) (.*);" : r"\1auto \4 = std::make_shared<std_msgs::msg::\3>();",
    }

    #Warnings for replacements that may require changes elsewhere in the code
    #Pattern:Warning message
    REPLACEMENT_WARNINGS = {
    r"(#include .*)h(>|\").*" : "ROS header files should all be .hpp",
    r"nsec" : "nsec has been replaced with nanosec",
    }

    #Warnings for boost replacements
    BOOST_REPLACEMENTS = {
        r"<boost/shared_ptr.hpp>" : r"<memory>",
        r"boost::shared_ptr" : r"std::shared_ptr",
        r"boost::mutex::scoped_lock" : r"std::unique_lock<std::mutex>",
        r"boost::mutex" : r"std::mutex",
        r"#include <boost/thread/mutex.hpp>" : r"#include <mutex>",
        r"#include <boost/unordered_map.hpp>" : r"#include <unorder_map>",
        r"boost::unordered_map" : r"std::unordered_map",
        r"#include<boost/function.hpp>" : r"#include <functional>",
        r"boost::function" : "std::function",
    }






