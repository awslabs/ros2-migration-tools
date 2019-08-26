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

import xml.etree.ElementTree as etree

from porting_tools.package_xml_porter import PackageXMLPorter


class TestPackageXMLPorter(unittest.TestCase):

    def test_rule_set_format(self):
        tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                    '<package format="%s">\n'
                    '</package>')
        before = etree.fromstring(tree_xml % "2")
        after = etree.fromstring(tree_xml % "3")
        PackageXMLPorter.rule_set_format(before)
        self.assertEqual(etree.tostring(before), etree.tostring(after))

    def test_rule_set_build_tool(self):
        # Test replacing catkin build tool
        tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                    '<package format="2">\n'
                    '<buildtool_depend>%s</buildtool_depend>\n'
                    '</package>')
        before = etree.fromstring(tree_xml % "catkin")
        after = etree.fromstring(tree_xml % "ament_cmake")
        PackageXMLPorter.rule_set_build_tool(before)
        self.assertEqual(etree.tostring(before), etree.tostring(after))

    def test_rule_set_build_tool_create(self):
        # Test adding ament as a build tool
        tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                    '<package format="2">%s\n'
                    '</package>')
        before = etree.fromstring(tree_xml % "")
        after = etree.fromstring(tree_xml % "\n<buildtool_depend>ament_cmake</buildtool_depend>")
        PackageXMLPorter.rule_set_build_tool(before)
        self.assertEqual(etree.tostring(before), etree.tostring(after))

    def test_rule_set_build_tool_dont_remove(self):
        # Test that buildtool dependencies other than catkin aren't removed
        tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                    '<package format="2">\n'
                    '<buildtool_depend>other</buildtool_depend>%s\n'
                    '</package>')
        before = etree.fromstring(tree_xml % "")
        after = etree.fromstring(tree_xml % "\n<buildtool_depend>ament_cmake</buildtool_depend>")
        PackageXMLPorter.rule_set_build_tool(before)
        self.assertEqual(etree.tostring(before), etree.tostring(after))

    def test_rule_set_client_library_cpp(self):
        tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                    '<package format="2">\n'
                    '<run_depend>%s</run_depend>\n'
                    '</package>')
        before = etree.fromstring(tree_xml % "roscpp")
        after = etree.fromstring(tree_xml % "rclcpp")
        PackageXMLPorter.rule_set_client_library(before)
        self.assertEqual(etree.tostring(before), etree.tostring(after))

    def test_rule_set_client_library_python(self):
        tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                    '<package format="2">\n'
                    '<run_depend>%s</run_depend>\n'
                    '</package>')
        before = etree.fromstring(tree_xml % "rospy")
        after = etree.fromstring(tree_xml % "rclpy")
        PackageXMLPorter.rule_set_client_library(before)
        self.assertEqual(etree.tostring(before), etree.tostring(after))

    def test_rule_add_export_build_type(self):
        tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                    '<package format="2">%s\n'
                    '</package>')
        before = etree.fromstring(tree_xml % "")
        after = etree.fromstring(tree_xml % "\n<export>\n    <build_type>ament_cmake</build_type>\n  </export>")
        PackageXMLPorter.rule_add_export_build_type(before)
        self.assertEqual(etree.tostring(before), etree.tostring(after))

    def test_rule_set_depend_to_run_exec(self):
        tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                    '<package format="2">\n'
                    '<%sdepend>dep</%sdepend>\n'
                    '%s'
                    '</package>')
        before = etree.fromstring(tree_xml % ("","",""))
        after = etree.fromstring(tree_xml % ("build_", "build_", "<exec_depend>dep</exec_depend>\n"))
        PackageXMLPorter.rule_set_depend_to_run_exec(before)
        self.assertEqual(etree.tostring(before), etree.tostring(after))

    def test_rule_set_run_to_exec_depend_change(self):
        tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                    '<package format="2">\n'
                    '<%s_depend>dep</%s_depend>\n'
                    '</package>')
        before = etree.fromstring(tree_xml % ("run","run"))
        after = etree.fromstring(tree_xml % ("exec", "exec"))
        PackageXMLPorter.rule_set_run_to_exec_depend(before)
        self.assertEqual(etree.tostring(before), etree.tostring(after))

    def test_rule_set_run_to_exec_depend_no_change(self):
        tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                    '<package format="2">\n'
                    '<build_depend>dep</build_depend>\n'
                    '</package>')
        before = etree.fromstring(tree_xml)
        after = etree.fromstring(tree_xml)
        PackageXMLPorter.rule_set_run_to_exec_depend(before)
        self.assertEqual(etree.tostring(before), etree.tostring(after))

    def test_rule_update_message_gen_dependency_unused(self):
        tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                    '<package format="2">\n'
                    '<build_depend>dep</build_depend>\n'
                    '</package>')
        before = etree.fromstring(tree_xml)
        after = etree.fromstring(tree_xml)
        PackageXMLPorter.rule_update_message_gen_dependency(before)
        self.assertEqual(etree.tostring(before), etree.tostring(after))

    def test_rule_update_message_gen_dependency_used(self):
        before_tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                           '<package format="2">\n'
                           '<build_depend>message_generation</build_depend>\n'
                           '<exec_depend>message_runtime</exec_depend>\n'
                           '</package>')
        after_tree_xml = ("<?xml version='1.0' encoding='utf-8'?>\n"
                          '<package format="2">\n'
                          '<buildtool_depend>rosidl_default_generators</buildtool_depend>\n'
                          '<build_depend>builtin_interfaces</build_depend>\n'
                          '<exec_depend>builtin_interfaces</exec_depend>\n'
                          '<exec_depend>rosidl_default_runtime</exec_depend>\n'
                          '<member_of_group>rosidl_interface_packages</member_of_group>\n'
                          '</package>')
        before = etree.fromstring(before_tree_xml)
        after = etree.fromstring(after_tree_xml)
        PackageXMLPorter.rule_update_message_gen_dependency(before)
        self.assertEqual(etree.tostring(before), etree.tostring(after))


if __name__ == '__main__':
    unittest.main()
