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


import unittest

import parse_cmake.parsing as cmkp

from porting_tools.cmake_lists_porter import CMakeListsPorter


class TestCmakeListsPorter(unittest.TestCase):

    def test_rule_set_cmake_version_change(self):
        before_cmake = [cmkp.Command("cmake_minimum_required", [cmkp.Arg("VERSION"), cmkp.Arg("2.5")])]
        after_cmake = [cmkp.Command("cmake_minimum_required", [cmkp.Arg("VERSION"), cmkp.Arg("3.5")])]
        CMakeListsPorter.rule_set_cmake_version(before_cmake)
        self.assertEqual(before_cmake, after_cmake)

    def test_rule_set_cmake_version_add(self):
        before_cmake = []
        after_cmake = [cmkp.Command("cmake_minimum_required", [cmkp.Arg("VERSION"), cmkp.Arg("3.5")])]
        CMakeListsPorter.rule_set_cmake_version(before_cmake)
        self.assertEqual(before_cmake, after_cmake)

    def test_cpp_std_definition(self):
        before_cmake = [cmkp.Command("add_definitions", [cmkp.Arg("-std=c++11")])]
        after_cmake = [cmkp.Command("add_definitions", [cmkp.Arg("-std=c++14")])]
        self.assertTrue(CMakeListsPorter.set_cpp_std_definition(before_cmake))
        self.assertEqual(before_cmake, after_cmake)

    def test_set_cpp_std_flag_1(self):
        before_cmake = [cmkp.Command("set", [cmkp.Arg("-std=c++11")])]
        after_cmake = [cmkp.Command("set", [cmkp.Arg("-std=c++14")])]
        self.assertTrue(CMakeListsPorter.set_cpp_std_flag(before_cmake))
        self.assertEqual(before_cmake, after_cmake)

    def test_set_cpp_std_flag_2(self):
        before_cmake = [cmkp.Command("set", [cmkp.Arg("CMAKE_CXX_STANDARD"),cmkp.Arg("11")])]
        after_cmake = [cmkp.Command("set", [cmkp.Arg("CMAKE_CXX_STANDARD"),cmkp.Arg("14")])]
        self.assertTrue(CMakeListsPorter.set_cpp_std_flag(before_cmake))
        self.assertEqual(before_cmake, after_cmake)

    def test_rule_set_cpp_std(self):
        before_cmake = [cmkp.Command("project", [cmkp.Arg("project_name")])]
        after_cmake = [cmkp.Command("project", [cmkp.Arg("project_name")]),
                       cmkp.BlankLine(),
                       cmkp.Comment("# Add support for C++14"),
                       cmkp.Command("if", [cmkp.Arg("NOT"), cmkp.Arg("CMAKE_CXX_STANDARD")]),
                       cmkp.Command("set", [cmkp.Arg("CMAKE_CXX_STANDARD"), cmkp.Arg("14")]),
                       cmkp.Command("endif", []),
                       cmkp.BlankLine()
                       ]
        CMakeListsPorter.rule_set_cpp_std(before_cmake)
        self.assertEqual(before_cmake, after_cmake)

    def test_get_catkin_dependencies(self):
        cmake = [cmkp.Command("find_package", [cmkp.Arg("COMPONENTS"),
                                               cmkp.Arg("roslib"),  # no longer in ros2
                                               cmkp.Arg("roscpp"),  # renamed to rclcpp
                                               cmkp.Arg("rosidl_default_runtime"),  # doesn't need to be found
                                               cmkp.Arg("message_generation"),  # require messages to be added
                                               cmkp.Arg("roscpp"),  # check duplicates
                                               ])]
        # Getting catkin_dependencies uses a set to only keep unique elements
        # This removes guarantees about order
        result = set(["ament_cmake",
                      "rclcpp",
                      "rosidl_default_generators"])
        self.assertEqual(set(CMakeListsPorter.get_catkin_dependencies(cmake)), result)

    def test_add_ament_dependencies(self):
        cmake = []
        dependencies = ["rclcpp"]
        after_cmake = [cmkp.Command("find_package", [cmkp.Arg("rclcpp"), cmkp.Arg(contents="REQUIRED")]),
                       cmkp.Command("set", [cmkp.Arg("INCLUDE_DIRS"), cmkp.Arg("${rclcpp_INCLUDE_DIRS}")]),
                       cmkp.Command("include_directories", [cmkp.Arg("${INCLUDE_DIRS}")]),
                       cmkp.Command("set", [cmkp.Arg("LIBRARY_DIRS"), cmkp.Arg("${rclcpp_LIBRARIES}")]),
                       cmkp.Command("set", [cmkp.Arg("LIBS"), cmkp.Arg("${rclcpp_LIBRARIES}")])]
        # The next index to insert at should be 5
        self.assertEqual(CMakeListsPorter.add_ament_dependencies(cmake, dependencies, 0), 5)
        self.assertEqual(cmake, after_cmake)

    def test_update_messages_services(self):
        before_cmake = [cmkp.Command("add_message_files", [cmkp.Arg("FILES"), cmkp.Arg("message")]),
                        cmkp.Command("add_service_files", [cmkp.Arg("FILES"), cmkp.Arg("service")])]
        pkgs = ["std_msgs"]
        after_cmake = [cmkp.Command("rosidl_generate_interfaces", [cmkp.Arg("${PROJECT_NAME}"),
                                                                   cmkp.Arg('"msg/message"'),
                                                                   cmkp.Arg('"srv/service"'),
                                                                   cmkp.Arg("DEPENDENCIES"),
                                                                   cmkp.Arg("builtin_interfaces"),
                                                                   cmkp.Arg("std_msgs")])]
        CMakeListsPorter.update_messages_services(before_cmake, pkgs, 0)
        self.assertEqual(before_cmake, after_cmake)

    def test_delete_commands_with(self):
        before_cmake = [cmkp.Command("find_package", [cmkp.Arg("rclcpp"), cmkp.Arg("REQUIRED")]),
                        cmkp.Command("set", [cmkp.Arg("INCLUDE_DIRS"), cmkp.Arg("${rclcpp_INCLUDE_DIRS}")]),
                        cmkp.Command("include_directories", [cmkp.Arg("${INCLUDE_DIRS}")]),
                        cmkp.Command("set", [cmkp.Arg("LIBRARY_DIRS"), cmkp.Arg("${rclcpp_LIBRARIES}")]),
                        cmkp.Command("set", [cmkp.Arg("LIBS"), cmkp.Arg("${rclcpp_LIBRARIES}")])]
        after_cmake = [cmkp.Command("include_directories", [cmkp.Arg("${INCLUDE_DIRS}")])]
        CMakeListsPorter.delete_commands_with(names=("set", "find_package"), from_cmake=before_cmake)
        self.assertEqual(before_cmake, after_cmake)

    def test_rule_update_dependencies_find_package(self):
        before_cmake = [cmkp.Command("project", [cmkp.Arg("proj")]),
                        cmkp.Command("find_package", [cmkp.Arg("roscpp")]),]
        after_cmake = [cmkp.Command("project", [cmkp.Arg("proj")]),
                       cmkp.Command("find_package", [cmkp.Arg("ament_cmake"), cmkp.Arg("REQUIRED")]),
                       cmkp.Command("find_package", [cmkp.Arg("rclcpp"), cmkp.Arg("REQUIRED")]),
                       cmkp.Command("set", [cmkp.Arg("INCLUDE_DIRS"), cmkp.Arg("${ament_cmake_INCLUDE_DIRS}"), cmkp.Arg("${rclcpp_INCLUDE_DIRS}")]),
                       cmkp.Command("include_directories", [cmkp.Arg("${INCLUDE_DIRS}")]),
                       cmkp.Command("set", [cmkp.Arg("LIBRARY_DIRS"), cmkp.Arg("${ament_cmake_LIBRARIES}"), cmkp.Arg("${rclcpp_LIBRARIES}")]),
                       cmkp.Command("set", [cmkp.Arg("LIBS"), cmkp.Arg("${ament_cmake_LIBRARIES}"), cmkp.Arg("${rclcpp_LIBRARIES}")]),
                       cmkp.BlankLine(),
                       cmkp.Command("ament_export_dependencies", [cmkp.Arg("ament_cmake")]),
                       cmkp.Command("ament_export_dependencies", [cmkp.Arg("rclcpp")])]
        CMakeListsPorter.rule_update_dependencies(before_cmake)
        self.assertEqual(before_cmake, after_cmake)

    def test_rule_update_dependencies_execute(self):
        before_cmake = [cmkp.Command("project", [cmkp.Arg("proj")]),
                        cmkp.Command("find_package", [cmkp.Arg("roscpp")]),
                        cmkp.Command("add_executable", [cmkp.Arg("exec")])]
        after_cmake = [cmkp.Command("project", [cmkp.Arg("proj")]),
                       cmkp.Command("add_executable", [cmkp.Arg("exec")]),
                       cmkp.Command("find_package", [cmkp.Arg("ament_cmake"), cmkp.Arg("REQUIRED")]),
                       cmkp.Command("find_package", [cmkp.Arg("rclcpp"), cmkp.Arg("REQUIRED")]),
                       cmkp.Command("set", [cmkp.Arg("INCLUDE_DIRS"), cmkp.Arg("${ament_cmake_INCLUDE_DIRS}"), cmkp.Arg("${rclcpp_INCLUDE_DIRS}")]),
                       cmkp.Command("include_directories", [cmkp.Arg("${INCLUDE_DIRS}")]),
                       cmkp.Command("set", [cmkp.Arg("LIBRARY_DIRS"), cmkp.Arg("${ament_cmake_LIBRARIES}"), cmkp.Arg("${rclcpp_LIBRARIES}")]),
                       cmkp.Command("target_link_libraries", [cmkp.Arg("exec"), cmkp.Arg("${LIBRARY_DIRS}")]),
                       cmkp.Command("set", [cmkp.Arg("LIBS"), cmkp.Arg("${ament_cmake_LIBRARIES}"), cmkp.Arg("${rclcpp_LIBRARIES}")]),
                       cmkp.BlankLine(),
                       cmkp.Command("ament_export_dependencies", [cmkp.Arg("ament_cmake")]),
                       cmkp.Command("ament_export_dependencies", [cmkp.Arg("rclcpp")])]
        CMakeListsPorter.rule_update_dependencies(before_cmake)
        self.assertEqual(before_cmake, after_cmake)

    def test_rule_update_dependencies_project(self):
        before_cmake = [cmkp.Command("project", [cmkp.Arg("proj")]),]
        after_cmake = [cmkp.Command("project", [cmkp.Arg("proj")]),
                       cmkp.Command("find_package", [cmkp.Arg("ament_cmake"), cmkp.Arg("REQUIRED")]),
                       cmkp.Command("set", [cmkp.Arg("INCLUDE_DIRS"), cmkp.Arg("${ament_cmake_INCLUDE_DIRS}")]),
                       cmkp.Command("include_directories", [cmkp.Arg("${INCLUDE_DIRS}")]),
                       cmkp.Command("set", [cmkp.Arg("LIBRARY_DIRS"), cmkp.Arg("${ament_cmake_LIBRARIES}")]),
                       cmkp.Command("set", [cmkp.Arg("LIBS"), cmkp.Arg("${ament_cmake_LIBRARIES}")]),
                       cmkp.BlankLine(),
                       cmkp.Command("ament_export_dependencies", [cmkp.Arg("ament_cmake")])]
        CMakeListsPorter.rule_update_dependencies(before_cmake)
        self.assertEqual(before_cmake, after_cmake)

    def test_rule_add_export_libs(self):
        before_cmake = [cmkp.Command("add_library", [cmkp.Arg("lib")])]
        after_cmake = [cmkp.Command("add_library", [cmkp.Arg("lib")]),
                       cmkp.Command("ament_export_include_directories", [cmkp.Arg("${INCLUDE_DIRS}")]),
                       cmkp.Command("ament_export_libraries", [cmkp.Arg("lib"), cmkp.Arg("${LIBS}")]),
                       ]
        CMakeListsPorter.rule_add_export_libs(before_cmake)
        self.assertEqual(before_cmake, after_cmake)

    def test_rule_replace_target_link_libs(self):
        before_cmake = [cmkp.Command("target_link_libraries", [cmkp.Arg("${catkin_LIBRARIES}")])]
        after_cmake = [cmkp.Command("target_link_libraries", [cmkp.Arg("${LIBS}")])]
        CMakeListsPorter.rule_replace_target_link_libs(before_cmake)
        self.assertEqual(before_cmake, after_cmake)


if __name__ == '__main__':
    unittest.main()



