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

"""
For usage run:
$ python3 catkin_to_ament.py -h

"""


import os
import re
import argparse
import tempfile
import os.path
import xml.etree.ElementTree as etree

from .package_xml_porter import PackageXMLPorter 
from .cmake_lists_porter import CMakeListsPorter 

from .constants import CatkinToAmentMigration


def get_arg_parser():
    parser = argparse.ArgumentParser(description="Does initial work to convert CMakeLists and package.xml from ROS1 to ROS2. Unless otherwise specified, will use package.xml and CMakeLists.txt in the current file and output to ros2_[CMakeLists.txt, package.xml]")
    parser.add_argument("--src-xml", default=CatkinToAmentMigration.DEFAULT_SRC_PACKAGE_XML, type=str, help="Path to ROS1 package.xml file")
    parser.add_argument("--dst-xml", default=CatkinToAmentMigration.DEFAULT_DST_PACKAGE_XML, type=str, help="Path to write ROS2 package.xml file")
    parser.add_argument("--src-cmake", default=CatkinToAmentMigration.DEFAULT_SRC_CMAKE_LISTS, type=str, help="Path to ROS1 CMakeLists.txt file")
    parser.add_argument("--dst-cmake", default=CatkinToAmentMigration.DEFAULT_DST_CMAKE_LISTS, type=str, help="Path to write ROS2 CMakeLists.txt file")
    parser.add_argument("--package-path", default=None, type=str, help="If provided, this path will override manually set source/destination files and will overwrite the CMakeLists.txt and package.xml files in the given package.")
    return parser

def get_file_locations(src_xml, dst_xml, src_cmake, dst_cmake, package_path):
    """
    Returns paths to the package.xml and CMakeLists source and destination files based on args
    """
    src_xml_path = dst_xml_path = src_cmake_path = dst_cmake_path = None
    if package_path is not None:
        if not os.path.exists(package_path):
            print("Package: %s path does not exist" % package_path)
            raise FileNotFoundError(package_path)
        if os.path.exists(os.path.join(package_path, "package.xml")):
            src_xml_path = dst_xml_path = os.path.join(package_path, "package.xml")
        if os.path.exists(os.path.join(package_path, "CMakeLists.txt")):
            src_cmake_path = dst_cmake_path = os.path.join(package_path, "CMakeLists.txt")
    else:
        if os.path.exists(os.path.abspath(src_xml)):
            src_xml_path = os.path.abspath(src_xml)
        if os.path.exists(os.path.dirname(os.path.abspath(dst_xml))):
            dst_xml_path = os.path.abspath(dst_xml)
        if os.path.exists(os.path.abspath(src_cmake)):
            src_cmake_path = os.path.abspath(src_cmake)
        if os.path.exists(os.path.dirname(os.path.abspath(dst_cmake))):
            dst_cmake_path = os.path.abspath(dst_cmake)

    return src_xml_path, dst_xml_path, src_cmake_path, dst_cmake_path


def catkin_to_ament(src_xml, dst_xml, src_cmake, dst_cmake, package_path):
    src_xml,dst_xml,src_cmake,dst_cmake = get_file_locations(src_xml,
                                                             dst_xml,
                                                             src_cmake,
                                                             dst_cmake,
                                                             package_path)

    ##Convert xml
    if src_xml is None:
        print("Invalid source file for package.xml")
    if dst_xml is None:
        print("Invalid destination file for package.xml")
    if src_xml is not None and dst_xml is not None:
        print("Porting package.xml")
        tree = etree.parse(src_xml)
        PackageXMLPorter.port(tree=tree)
        tree.write(dst_xml, encoding="utf-8", xml_declaration=True)

    ##Convert CMakeLists
    if src_cmake is None:
        print("Invalid source file for CMakeLists")
    if dst_cmake is None:
        print("Invalid destination file for CMakeLists")
    if src_cmake is not None and dst_cmake is not None:
        print("Porting CMakeLists")
        with open(src_cmake, 'r') as src_file:
            ported_content = CMakeListsPorter.port(content=src_file.read())
        with open(dst_cmake, 'w') as dst_file:
            dst_file.write(ported_content)


def main():
    args = get_arg_parser().parse_args()
    catkin_to_ament(args.src_xml,
                    args.dst_xml,
                    args.src_cmake,
                    args.dst_cmake,
                    args.package_path)
    



if __name__ == '__main__':
    main()
