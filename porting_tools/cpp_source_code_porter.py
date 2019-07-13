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


class CPPSourceCodePorter:
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
        CPPSourceCodePorter.NODE_NAME = None
        CPPSourceCodePorter.NODE_VAR_NAME = None
        CPPSourceCodePorter.POINTER_VARIABLES = []

    @staticmethod
    def port(source, mapping, ast):
        """
        Makes some automatic changes to ROS1 source code to port to ROS2

        Arguments:
            source - A string of the c++ source code
        Returns:
            The new source code
        """
        CPPSourceCodePorter.init()

        src_lines = source.split('\n')
        new_source = []
        for line_number in range(len(src_lines)):
            # actual line number in the file will be start from 1, so use line_number + 1
            new_source.append(CPPSourceCodePorter.port_line(src_lines[line_number], line_number + 1, mapping, ast))

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
        if "#include" in line:
            line = CPPSourceCodePorter.rule_replace_headers(line, mapping)
        else:
            line = CPPSourceCodePorter.rule_replace_var_decl(line, line_number, mapping, ast)
            line = CPPSourceCodePorter.rule_replace_call_expr(line, line_number, mapping, ast)
            line = CPPSourceCodePorter.rule_replace_macros(line, line_number, mapping, ast)
            line = CPPSourceCodePorter.rule_replace_namespace_ref(line, mapping)
            line = CPPSourceCodePorter.rule_replace_dot_with_arrow(line)
            line = CPPSourceCodePorter.rule_dereference_pointers(line)

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
            if Constants.ROS_2_NAME in mapping[ros1_name]:
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
    def get_line_token_with_new_arg(line_tokens, arg_ind, arg_name):
        """
        Inserts arg_name as an argument at arg_ind and returns the new list of tokens
        :param line_tokens: list containing old tokens
        :param arg_ind: argument index of NODE_VAR_NAME, zero-based indexing is used
        :param arg_name: name of the argument to be added at arg_ind
        :return: list
        """

        # line_tokens should atleast be ["fun", "(", ")"]
        if len(line_tokens) < 3:
            raise Exception("line tokens list not valid")
        if arg_ind < 0:
            raise Exception("node argument index can't be negative")

        new_line_tokens = []
        req_comma_cnt = arg_ind

        ind = 0
        bracket_found = False
        while ind < len(line_tokens):

            if req_comma_cnt == 0 and bracket_found:
                new_line_tokens.append(arg_name)
                break

            if ind == len(line_tokens) - 1:
                new_line_tokens.append(",")
                new_line_tokens.append(arg_name)
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

        return new_line_tokens

    @staticmethod
    def handle_var_creation_method(token, line, mapping):
        """
        Handles different ways of VAR_DECL
        :param token: token containing the var info
        :param line: source code line
        :param mapping: ROS1 to ROS2 mapping of VAR_DECL
        :return:
        """
        var_name = token[AstConstants.NAME]
        var_type = token[AstConstants.VAR_TYPE]

        try:
            to_shared_ptr = mapping[var_type][Constants.TO_SHARED_PTR]
        except KeyError as e:
            raise Exception(e.message + ": key not found")

        if to_shared_ptr:
            CPPSourceCodePorter.POINTER_VARIABLES.append(var_name)

        ros2_name = CPPSourceCodePorter.get_ros2_name(var_type, mapping)

        line_pattern = var_type + " *" + var_name + " *="
        if re.search(line_pattern, line) is not None:
            pattern = var_type + " *" + var_name
            replacement = "auto " + var_name
        else:
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
        :param mapping: ros1 to ros2 mapping for various types
        :return: str
        """
        header_mapping = mapping[AstConstants.INCLUSION_DIRECTIVE]
        for header in header_mapping:
            if header in line:
                line = line.replace(header, header_mapping[header][Constants.ROS_2_NAME])
        return line

    @staticmethod
    def rule_replace_namespace_ref(line, mapping):
        """
        Changes the ros1 namespace to corresponding ros2 namespaces
        :param line: source line
        :param mapping: ros1 to ros2 mapping for namespace_references
        :return: new source str
        """
        namespace_mapping = mapping[AstConstants.NAMESPACE_REF]
        for to_replace in namespace_mapping:
            if to_replace != Constants.NEW_TOKENS_LIST:
                replace_with = namespace_mapping[to_replace][Constants.ROS_2_NAME] + "::"
                to_replace += "::"
                line = line.replace(to_replace, replace_with)
        return line

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
            token = CPPSourceCodePorter.find_token_in_ast(AstConstants.NAME, RosConstants.INIT_CALL_EXPR,
                                                          ast[line_number][AstConstants.CALL_EXPR])
            if token:
                line_tokens = token[AstConstants.LINE_TOKENS]
                # tokens for init:  ['ros', '::', 'init', '(', 'argc', ',', 'argv', ',', '"talker"', ')']
                # so min length should be 10
                if len(line_tokens) >= 10:
                    CPPSourceCodePorter.NODE_NAME = line_tokens[8]

                    ros2_name = CPPSourceCodePorter.get_ros2_name(line_tokens[2], mapping)

                    init_pattern = RosConstants.INIT_PATTERN + ".*\)"
                    replace_with = line_tokens[0] + line_tokens[1] + ros2_name + \
                        line_tokens[3] + line_tokens[4] + line_tokens[5] + line_tokens[6] + ")"
                    return re.sub(init_pattern, replace_with, line)
                else:
                    raise Exception("line tokens for ros::init invalid")
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

        init_line = CPPSourceCodePorter.rule_init_call_found(line, line_number, mapping, ast)
        if init_line:
            return init_line
        else:
            call_expr_mapping = mapping[AstConstants.CALL_EXPR]
            for fun_call in call_expr_mapping:
                if fun_call in line:
                    if Constants.NODE_ARG_INFO in call_expr_mapping[fun_call]:
                        node_arg_info = call_expr_mapping[fun_call][Constants.NODE_ARG_INFO]
                        if Constants.NODE_ARG_REQ in node_arg_info and node_arg_info[Constants.NODE_ARG_REQ]:
                            if Constants.NODE_ARG_INDEX not in node_arg_info:
                                raise Exception(Constants.NODE_ARG_INDEX + " key missing")
                            node_arg_ind = node_arg_info[Constants.NODE_ARG_INDEX]
                            token = CPPSourceCodePorter.find_token_in_ast(AstConstants.NAME, fun_call,
                                                                          ast[line_number][AstConstants.CALL_EXPR])
                            if token:
                                line_tokens = token[AstConstants.LINE_TOKENS]

                                if CPPSourceCodePorter.NODE_VAR_NAME is None:
                                    raise Exception("Node name missing")

                                new_line_token = CPPSourceCodePorter.get_line_token_with_new_arg(line_tokens,
                                                                                                 node_arg_ind,
                                                                                                 CPPSourceCodePorter.
                                                                                                 NODE_VAR_NAME)
                                pattern = "(ros::)?" + fun_call + "\(.*\)"
                                replacement = "".join(new_line_token)

                                line = re.sub(pattern, replacement, line)
                                line = line.replace(fun_call, call_expr_mapping[fun_call][Constants.ROS_2_NAME])
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

            token = CPPSourceCodePorter.find_token_in_ast(AstConstants.VAR_TYPE, RosConstants.NODE_HANDLE,
                                                          ast[line_number][AstConstants.VAR_DECL])
            if token:
                var_name = token[AstConstants.NAME]
                to_shared_ptr = mapping[RosConstants.NODE_HANDLE][Constants.TO_SHARED_PTR]
                if to_shared_ptr:
                    CPPSourceCodePorter.POINTER_VARIABLES.append(var_name)

                CPPSourceCodePorter.NODE_VAR_NAME = var_name

                pattern = RosConstants.NODE_HANDLE + " *" + var_name

                replacement = "auto " + var_name + " = " + CPPSourceCodePorter.get_ros2_name(RosConstants.NODE_HANDLE,
                                mapping) + "::make_shared(" + CPPSourceCodePorter.NODE_NAME + ")"

                return re.sub(pattern, replacement, line)

        return None

    @staticmethod
    def rule_replace_var_decl(line, line_number, mapping, ast):
        """
        Changes the ros1 var declarations to corresponding ros2 var decl
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: str
        """
        var_decl_mapping = mapping[AstConstants.VAR_DECL]
        new_line = CPPSourceCodePorter.rule_node_handle_found(line, line_number, var_decl_mapping, ast)
        if new_line is not None:
            return new_line
        else:
            for var_type in var_decl_mapping:
                if var_type in line:
                    if line_number in ast:
                        token = CPPSourceCodePorter.find_token_in_ast(AstConstants.VAR_TYPE, var_type,
                                                                      ast[line_number][AstConstants.VAR_DECL])
                        if token:
                            line = CPPSourceCodePorter.handle_var_creation_method(token, line, var_decl_mapping)
        return line

    @staticmethod
    def rule_replace_dot_with_arrow(line):
        """
        Replaces all c++ '.' calls with '->' calls for pointer variables
        :param line: source line
        :return: str
        """
        for var_name in CPPSourceCodePorter.POINTER_VARIABLES:
            pattern = "[^a-zA-Z0-9_]" + var_name + "\."
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
        :return: str
        """

        # ToDO: write regex for finding pointers which are required to be deferenced
        return line

    @staticmethod
    def rule_replace_macros(line, line_number, mapping, ast):
        """
        Changes the ros1 macros to corresponding ros2 macros
        :param line: source line
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: str
        """
        macros_mapping = mapping[AstConstants.MACRO_INSTANTIATION]
        for macro in macros_mapping:
            if macro in line:
                if Constants.NODE_ARG_INFO in macros_mapping[macro]:
                    node_arg_info = macros_mapping[macro][Constants.NODE_ARG_INFO]
                    if Constants.NODE_ARG_REQ in node_arg_info and node_arg_info[Constants.NODE_ARG_REQ]:
                        if Constants.NODE_ARG_INDEX not in node_arg_info:
                            raise Exception(Constants.NODE_ARG_INDEX + " key missing")

                        member_name = None
                        if Constants.MEMBER_NAME_IF_TRUE in node_arg_info:
                            member_name = node_arg_info[Constants.MEMBER_NAME_IF_TRUE]

                        node_arg_ind = node_arg_info[Constants.NODE_ARG_INDEX]
                        token = CPPSourceCodePorter.find_token_in_ast(AstConstants.NAME, macro,
                                                                      ast[line_number][AstConstants.MACRO_INSTANTIATION])
                        if token:
                            line_tokens = token[AstConstants.LINE_TOKENS]
                            if member_name is not None:
                                arg_name = CPPSourceCodePorter.NODE_VAR_NAME + "->" + member_name + "()"
                                new_line_token = CPPSourceCodePorter.get_line_token_with_new_arg(line_tokens,
                                                                                                 node_arg_ind, arg_name)
                                pattern = macro + "\(.*\)"
                                replacement = "".join(new_line_token)

                                line = re.sub(pattern, replacement, line)
                            line = line.replace(macro, macros_mapping[macro][Constants.ROS_2_NAME])
                    else:
                        line = line.replace(macro, macros_mapping[macro][Constants.ROS_2_NAME])
        return line

    def __init__(self):
        pass
