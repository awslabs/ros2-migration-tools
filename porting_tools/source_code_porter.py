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
from Constants import AstConstants, Constants, RosConstants
from utilities import Utilities


class SourceCodePorter():
    """ Class containing static methods to change and print warning on C++ source code """
    NODE_NAME = None
    NODE_VAR_NAME = None
    POINTER_VARIABLES = None

    @staticmethod
    def init():
        """
        Initializes the variables to initial values
        :return: None
        """
        SourceCodePorter.NODE_NAME = None
        SourceCodePorter.NODE_VAR_NAME = None
        SourceCodePorter.POINTER_VARIABLES = []

    @staticmethod
    def port(source, mapping, ast):
        """
        Makes some automatic changes to ROS1 source code to port to ROS2

        Arguments:
            source - A string of the c++ source code
        Returns:
            The new source code
        """
        SourceCodePorter.init()

        src_lines = source.split('\n')
        new_source = []
        for line_number in range(len(src_lines)):
            # actual line number in the file will be start from 1, so use line_number + 1
            new_source.append(SourceCodePorter.port_line(src_lines[line_number], line_number+1, mapping, ast))

        return "\n".join(new_source)

    @staticmethod
    def port_line(line, line_number, mapping, ast):
        """
        Finds tokens in each line and converts it to corresponding ROS2 token
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: new line str
        """
        line = SourceCodePorter.rule_replace_headers(line, mapping)
        line = SourceCodePorter.rule_replace_var_decl(line, line_number, mapping, ast)
        line = SourceCodePorter.rule_replace_call_expr(line, line_number, mapping, ast)
        line = SourceCodePorter.rule_replace_namespace_ref(line, mapping)
        line = SourceCodePorter.rule_replace_dot_with_arrow(line)
        line = SourceCodePorter.rule_dereference_pointers(line)
        line = SourceCodePorter.rule_replace_macros(line, mapping)
        return line

    @staticmethod
    def get_ros2_name(ros1_name, mapping):
        """
        Returns ros2_name if one found in mapping, otherwise returns the ros1_name
        :param ros1_name: name in ROS1
        :param mapping: ROS1 to ROS2 mapping
        :return: str
        """
        # use ros1_name if ros2_name not in mapping
        ros2_name = ros1_name
        if ros1_name in mapping:
            ros2_name = mapping[ros1_name][Constants.ROS_2_NAME]
        return ros2_name

    @staticmethod
    def find_token_in_ast(key, value, ast_of_type):
        """
        Find the token with key and value in ast_of_type
        :param key: key of token in ast_of_type
        :param value: value corresponding to key of token in ast_of_type
        :param ast_of_type: list of tokens
        :return: token dict
        """
        for token in ast_of_type:
            if token[key] == value:
                return token
        return None

    @staticmethod
    def get_line_token_with_node_param(line_tokens, node_arg_ind):
        """
        Inserts NODE_VAR_NAME as an argument and returns the new list of tokens
        :param line_tokens: list containing old tokens
        :param node_arg_ind: argument index of NODE_VAR_NAME, zero-based indexing is used
        :return: list
        """
        new_line_tokens = []
        req_comma_cnt = node_arg_ind

        try:
            ind = 0
            bracket_found = False
            while ind < len(line_tokens):

                if req_comma_cnt == 0 and bracket_found:
                    new_line_tokens.append(SourceCodePorter.NODE_VAR_NAME)
                    break

                if ind == len(line_tokens) - 1:
                    new_line_tokens.append(",")
                    new_line_tokens.append(SourceCodePorter.NODE_VAR_NAME)
                    break

                if line_tokens[ind] == ",":
                    req_comma_cnt -= 1

                if line_tokens[ind] == "(":
                    bracket_found = True

                new_line_tokens.append(line_tokens[ind])
                ind += 1

            if line_tokens[ind] != ')':
                new_line_tokens.append(",")

            while ind < len(line_tokens):
                new_line_tokens.append(line_tokens[ind])
                ind += 1
            assert node_arg_ind >= 0
        except BaseException as e:
            raise e

        return new_line_tokens

    @staticmethod
    def handle_var_creation_method(token, line, mapping):
        """
        Handles different ways of VAR_DECL
        :param token: token containgin the var info
        :param line: source code line
        :param mapping: ROS1 to ROS2 mapping of VAR_DECL
        :return:
        """
        var_name = token[AstConstants.NAME]
        var_type = token[AstConstants.VAR_TYPE]

        try:
            to_shared_ptr = mapping[var_type][Constants.TO_SHARED_PTR]
            is_created_by_node = mapping[var_type][Constants.CREATION_INFO][Constants.IS_CREATED_BY_NODE]
            node_member_name = mapping[var_type][Constants.CREATION_INFO][Constants.MEMBER_NAME_IF_TRUE]
        except AttributeError as e:
            raise e

        if to_shared_ptr:
            SourceCodePorter.POINTER_VARIABLES.append(var_name)

        if is_created_by_node:
            replacement = "auto " + var_name + " = " + SourceCodePorter.NODE_VAR_NAME + "->" + node_member_name
            if to_shared_ptr:
                pattern = var_type + " *" + var_name + ".*(?=<)"
            else:
                pattern = var_type + " *" + var_name + ".*(?=\()"
        else:
            ros2_name = SourceCodePorter.get_ros2_name(var_type, mapping)
            pattern = var_type + " *" + var_name + ";"
            if to_shared_ptr:
                replacement = "auto " + var_name + " = std::make_shared<" + ros2_name + ">();"
            else:
                replacement = ros2_name + " " + var_name + ";"

        line = re.sub(pattern, replacement, line)
        return line


    #########################
    #        RULES          #
    #########################

    @staticmethod
    def rule_replace_headers(line, mapping):
        """
        Changes the ros1 includes to corresponding ros2 includes
        :param line: source line
        :param mapping: ros1 to ros2 mapping for headers
        :return: new source str
        """
        header_mapping = mapping[Constants.INCLUDES]
        return Utilities.simple_replace(line, header_mapping)

    @staticmethod
    def rule_replace_namespace_ref(line, mapping):
        """
        Changes the ros1 namespace to corresponding ros2 namespaces
        :param line: source line
        :param mapping: ros1 to ros2 mapping for namespace_references
        :return: new source str
        """
        namespace_mapping = mapping[AstConstants.NAMESPACE_REF]
        return Utilities.simple_replace(line, namespace_mapping)

    @staticmethod
    def rule_init_call_found(line, line_number, mapping, ast):
        """
        If line contains ros::init call, then it will update SourcecodePorter.NODE_NAME and replace the call with
        ros2 init call
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: new line if init found, None if init not found
        """
        if line_number in ast and RosConstants.INIT_PATTERN in line:
            token = SourceCodePorter.find_token_in_ast(AstConstants.NAME, RosConstants.INIT_CALL_EXPR,
                                                       ast[line_number][AstConstants.CALL_EXPR])
            if token:
                line_tokens = token[AstConstants.LINE_TOKENS]
                # tokens for init:  ['ros', '::', 'init', '(', 'argc', ',', 'argv', ',', '"talker"', ')']
                # so min length should be 10
                if len(line_tokens) >= 10:
                    SourceCodePorter.NODE_NAME = line_tokens[8]

                    ros2_name = SourceCodePorter.get_ros2_name(line_tokens[2], mapping)

                    init_pattern = RosConstants.INIT_PATTERN + ".*\)"
                    replace_with = line_tokens[0] + line_tokens[1] + ros2_name + \
                        line_tokens[3] + line_tokens[4] + line_tokens[5] + line_tokens[6] + ")"
                    return re.sub(init_pattern, replace_with, line)
        return None

    @staticmethod
    def rule_replace_call_expr(line, line_number, mapping, ast):
        """
        Changes the ros1 function calls to corresponding ros2 function calls
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: new source str
        """

        init_line = SourceCodePorter.rule_init_call_found(line, line_number, mapping, ast)
        if init_line:
            return init_line
        else:
            call_expr_mapping = mapping[AstConstants.CALL_EXPR]
            for fun_call in call_expr_mapping:
                if fun_call in line:
                    if Constants.NODE_ARG_INFO in call_expr_mapping[fun_call]:
                        node_arg_info = call_expr_mapping[fun_call][Constants.NODE_ARG_INFO]
                        if Constants.NODE_ARG_REQ in node_arg_info and node_arg_info[Constants.NODE_ARG_REQ]:
                            try:
                                node_arg_ind = node_arg_info[Constants.NODE_ARG_INDEX]
                                token = SourceCodePorter.find_token_in_ast(AstConstants.NAME, fun_call,
                                                                           ast[line_number][AstConstants.CALL_EXPR])
                                line_tokens = token[AstConstants.LINE_TOKENS]
                                new_line_token = SourceCodePorter.get_line_token_with_node_param(line_tokens,
                                                                                                 node_arg_ind)
                                pattern = "(ros::)?" + fun_call + "\(.*\)"

                                replacement = "".join(new_line_token)

                                line = re.sub(pattern, replacement, line)
                                line = line.replace(fun_call, call_expr_mapping[fun_call][Constants.ROS_2_NAME])
                            except BaseException as e:
                                raise e
                        else:
                            line = line.replace(fun_call, call_expr_mapping[fun_call][Constants.ROS_2_NAME])
            return line

    @staticmethod
    def rule_node_handle_found(line, line_number,  mapping, ast):
        """
        When var type ros::NodeHandle is found, then line is modified for ROS2 and var name is saved in NODE_VAR_NAME
        which will be used by some CALL_EXPR
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: modified line is ros::NodeHandle found, None otherwise
        """
        if line_number in ast and RosConstants.NODE_HANDLE in line:
            try:
                token = SourceCodePorter.find_token_in_ast(AstConstants.VAR_TYPE, RosConstants.NODE_HANDLE,
                                                           ast[line_number][AstConstants.VAR_DECL])
                var_name = token[AstConstants.NAME]
                to_shared_ptr = mapping[RosConstants.NODE_HANDLE][Constants.TO_SHARED_PTR]
                if to_shared_ptr:
                    SourceCodePorter.POINTER_VARIABLES.append(var_name)

                SourceCodePorter.NODE_VAR_NAME = var_name

                pattern = RosConstants.NODE_HANDLE + " *" + var_name

                replacement = "auto " + var_name + " = " + SourceCodePorter.get_ros2_name(RosConstants.NODE_HANDLE,
                    mapping) + "::make_shared(" + SourceCodePorter.NODE_NAME + ")"

                return re.sub(pattern, replacement, line)
            except BaseException as e:
                raise e

        return None

    @staticmethod
    def rule_replace_var_decl(line, line_number, mapping, ast):
        """
        Changes the ros1 var declarations to corresponding ros2 var decl
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: new source str
        """
        var_decl_mapping = mapping[AstConstants.VAR_DECL]
        new_line = SourceCodePorter.rule_node_handle_found(line, line_number, var_decl_mapping, ast)
        if new_line is not None:
            return new_line
        else:
            for var_type in var_decl_mapping:
                if var_type in line:
                    if line_number in ast:
                        token = SourceCodePorter.find_token_in_ast(AstConstants.VAR_TYPE, var_type,
                                                                   ast[line_number][AstConstants.VAR_DECL])
                        if token:
                            line = SourceCodePorter.handle_var_creation_method(token, line, var_decl_mapping)
                            return line
        return line

    @staticmethod
    def rule_replace_dot_with_arrow(line):
        """
        Replaces all c++ '.' calls with '->' calls for pointer variables
        :param line: source line
        :return: new line
        """
        for var_name in SourceCodePorter.POINTER_VARIABLES:
            pattern = "[^a-zA-Z0-9]" + var_name + "\."
            sub_str = re.search(pattern, line)

            if sub_str:
                found_str = sub_str.group()
                to_replace = found_str[:-1] + "->"
                line = line.replace(found_str, to_replace)

        return line

    @staticmethod
    def rule_dereference_pointers(line):
        """
        Replaces all pointer calls with '->' calls for pointer variables
        :param line: source line
        :return: new line
        """

        # ToDO: write regex for finding pointers which are required to be deferenced
        return line

    @staticmethod
    def rule_replace_macros(line, mapping):
        """
        Changes the ros1 macros to corresponding ros2 macros
        :param line: source line
        :param mapping: ros1 to ros2 mapping for macros
        :return: str
        """
        macros_mapping = mapping[Constants.MACROS]
        return Utilities.simple_replace(line, macros_mapping)

    def __init__(self):
        pass