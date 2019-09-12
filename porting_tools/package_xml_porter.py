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

""" Contains a class and method for porting a package.xml file from catkin to ament"""
import xml.etree.ElementTree as etree

from .constants import CatkinToAmentMigration, PACKAGE_XML_ELEMENT_ORDER
from .utils import get_functions_with


def new_element(tag, text="", tail="\n", attrib=None):
    """ Helper function to make creating an element with a text and tail easier """
    if not attrib:
        attrib = {}
    element = etree.Element(tag, attrib=attrib)
    element.text = text
    element.tail = tail
    return element


def tag_order(tag):
    """ Returns integer to order tags """
    if tag in PACKAGE_XML_ELEMENT_ORDER:
        return PACKAGE_XML_ELEMENT_ORDER.index(tag)
    return float("inf")


class PackageXMLPorter:
    """A class for porting a package.xml file from catkin to ament"""
    @staticmethod
    def port(tree, extra_rules=[]):
        """
        Ports package.xml from catkin to ament
        Arguments:
            tree - the xml tree representing the package.xml file (output of etree.parse("package.xml"))
            extra_rules - a list of functions to apply to the xml tree
        Returns:
            The new xml tree
        """
        # Pulls out all methods in this class with name starting with "rule"
        rules = get_functions_with(criteria=lambda name: name.startswith("rule"),
                                   from_class=PackageXMLPorter)

        package_root = tree.getroot()

        for rule in rules + extra_rules:
            rule(package_root)

        # Make sure there's a final newline
        package_root.tail = "\n"

        # Reorder the elements
        package_root[:] = sorted(list(package_root), key=lambda elem: tag_order(elem.tag))

        # Correct indentation
        PackageXMLPorter.indent_tree(elem=package_root, level=0)

    #########################
    #        RULES          #
    #########################

    @staticmethod
    def rule_set_format(package_root):
        # ROS 2 supports formats 2,3
        package_root.set("format", "3")

    @staticmethod
    def rule_set_build_tool(package_root):
        for elem in package_root.findall("buildtool_depend"):
            if elem.text and elem.text.strip() == "catkin":
                package_root.remove(elem)
        package_root.append(new_element(tag="buildtool_depend", text="ament_cmake"))

    @staticmethod
    def rule_set_client_library(package_root):
        for elem in list(package_root):
            if elem.text and elem.text.strip() in CatkinToAmentMigration.CLIENT_CONVERSION:
                elem.text = CatkinToAmentMigration.CLIENT_CONVERSION[elem.text.strip()]

    @staticmethod
    def rule_add_export_build_type(package_root):
        build_elem = new_element(tag="build_type", text="ament_cmake", tail="\n  ")
        export_elem = new_element(tag="export", text="\n    ")
        export_elem.append(build_elem)
        package_root.append(export_elem)

    @staticmethod
    def rule_set_run_to_exec_depend(package_root):
        for elem in package_root.findall("run_depend"):
            elem.tag = "exec_depend"

    @staticmethod
    def rule_set_depend_to_run_exec(package_root):
        for elem in package_root.findall("depend"):
            elem.tag = "build_depend"
            package_root.append(new_element(tag="exec_depend", text=elem.text, attrib=elem.attrib))

    @staticmethod
    def rule_update_message_gen_dependency(package_root):
        message_generation_used = False
        for elem in list(package_root):
            if elem.text and elem.text == "message_generation" or elem.text == "message_runtime":
                package_root.remove(elem)
                message_generation_used = True
        if message_generation_used:
            package_root.append(new_element(tag="buildtool_depend", text="rosidl_default_generators"))
            package_root.append(new_element(tag="build_depend", text="builtin_interfaces"))
            package_root.append(new_element(tag="exec_depend", text="builtin_interfaces"))
            package_root.append(new_element(tag="exec_depend", text="rosidl_default_runtime"))
            package_root.append(new_element(tag="member_of_group", text="rosidl_interface_packages"))

    #########################
    #          HELPERS      #
    #########################

    @staticmethod
    def indent_tree(elem, level):
        if len(elem) > 0:  # element has children
            if elem.text is None or len(elem.text) == 0:
                elem.text = "\n" + ("  "*(level+1))  # sets the indent for the children
            list(elem)[-1].tail = "\n" + "  "*level
        for child in list(elem)[:-1]:
            child.tail = "\n" + ("  "*(level+1))
            PackageXMLPorter.indent_tree(elem=child, level=level+1)


if __name__ == '__main__':
    tree = etree.parse("package.xml")
    PackageXMLPorter.port(tree=tree)
    tree.write("updated_package.xml", encoding="utf-8", xml_declaration=True)
