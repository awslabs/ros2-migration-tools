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

""" A class and method for porting ROS1 source code to ROS2 """
import re
from .utils import get_functions_with
from Constants import AstConstants, Constants
from utilities import Utilities


class SourceCodePorter():
    """ Class containing static methods to change and print warning on C++ source code """
    @staticmethod
    def port(source, mapping):
        """
        Makes some automatic changes to ROS1 source code to port to ROS2

        Arguments:
            source - A string of the c++ source code
        Returns:
            The new source code
        """
        rules = get_functions_with(criteria=lambda name: name.startswith("rule"),
                                   from_class=SourceCodePorter)

        print(Utilities.get_uniquie_id())

        for rule in rules:
            source = rule(source, mapping)

        return source

    #########################
    #        RULES          #
    #########################

    @staticmethod
    def rule_replace_headers(source, mapping):
        """
        Changes the ros1 includes to corresponding ros2 includes
        :param source: source file content as string
        :param mapping: ros1 to ros2 mapping for headers
        :return: new source str
        """
        includes = mapping[Constants.INCLUDES]
        source = Utilities.simple_replace(source, includes)
        return source

    @staticmethod
    def rule_replace_namespace_ref(source, mapping):
        """
        Changes the ros1 namespace to corresponding ros2 namespaces
        :param source: source file content as string
        :param mapping: ros1 to ros2 mapping for namespace_references
        :return: new source str
        """
        namespaces = mapping[AstConstants.NAMPSPACE_REF]
        source = Utilities.simple_replace(source, namespaces)
        return source

    @staticmethod
    def rule_replace_call_expr(source, mapping):
        """
        Changes the ros1 function calls to corresponding ros2 function calls
        :param source: source file content as string
        :param mapping: ros1 to ros2 mapping for call_expr
        :return: new source str
        """
        namespaces = mapping[AstConstants.CALL_EXPR]
        source = Utilities.simple_replace(source, namespaces)
        return source


# def test():
#     print(Utilities.get_uniquie_id())
#
#
# if __name__ == "__main__":
#     test()