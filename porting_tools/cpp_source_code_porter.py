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
import logging
import os
import re

from clang.clang_parser import CppAstParser
from Constants import AstConstants, Constants, RosConstants
from utilities import Utilities


class CPPSourceCodePorter:
    """ Class containing static methods to change and print warning on C++ source code """

    def __init__(self, global_ast, curr_file_path):
        """
        Constructor for CPPSourceCodePorter
        :param global_ast: complete ast for the package
        ::param curr_file_path: current file_path
        """

        # flag to know if porting a unit test file
        self.is_porting_unit_test = Utilities.is_unit_test_path(curr_file_path)

        if not self.is_porting_unit_test:
            self.global_ast = global_ast[AstConstants.NON_UNIT_TEST]
            node_info = Utilities.get_ros_node_info(self.global_ast)
            self.NODE_NAME = node_info[Constants.NODE_NAME]
            self.NODE_VAR_NAME = node_info[Constants.NODE_HANDLE_VAR_NAME]
            self.NODE_VAR_PARENT_CLASS = node_info[Constants.NODE_VAR_PARENT_CLASS]
        else:
            self.global_ast = global_ast[AstConstants.UNIT_TEST]
            self.NODE_NAME = Utilities.get_node_name(self.global_ast)

        self.file_path = curr_file_path
        self.vars_to_be_removed = []
        self.template_from_call_back = {
            RosConstants.SERVICE_SERVER: self.get_ros_service_template,
            RosConstants.SUBSCRIBE: self.get_ros_subscribe_template
        }
        self.filter_out = Utilities.read_json_file(Utilities.get_abs_path(os.path.join("clang", Constants.FILTER_OUT_FILE_PATH)))

    def port(self, source, mapping, ast):
        """
        Makes changes to ROS1 source code to port to ROS2 and returns the ROS2 source code
        :param source: A string of the c++ source code
        :param mapping: ROS1 to ROS2 mapping for various token kinds
        :param ast: line by line AST
        :return: str
        """
        var_decl_mapping = mapping[AstConstants.VAR_DECL]
        for var_type in var_decl_mapping:
            if Constants.TO_BE_REMOVED in var_decl_mapping[var_type] and \
                    var_decl_mapping[var_type][Constants.TO_BE_REMOVED]:
                self.vars_to_be_removed.append(var_type)

        src_lines = source.split('\n')
        new_source = []
        for line_number in range(len(src_lines)):
            # actual line number in the file will be start from 1, so use line_number + 1
            new_source.append(self.port_line(src_lines[line_number], line_number + 1, mapping, ast))

        return "\n".join(new_source)

    def port_line(self, line, line_number, mapping, ast):
        """
        Finds tokens in each line and converts it to corresponding ROS2 token
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: str
        """
        if "#include" in line:
            line = self.rule_replace_headers(line, line_number, mapping)
        else:
            line = self.rule_replace_function_signature(line, line_number, mapping, ast)

            line = self.rule_replace_var_decl(line, line_number, mapping, ast)
            line = self.rule_replace_removed_var(line, line_number, mapping, ast)
            line = self.rule_replace_parm_decl(line, line_number, mapping, ast)
            line = self.rule_handle_var_instantiation(line, line_number, mapping, ast)
            line = self.rule_replace_conversion_function(line, line_number, mapping, ast)
            line = self.rule_replace_type_ref(line, line_number, mapping, ast)
            line = self.rule_replace_call_expr(line, line_number, mapping, ast)
            line = self.rule_replace_macros(line, line_number, mapping, ast)
            line = self.rule_replace_namespace_ref(line, line_number, mapping)
            line = self.rule_replace_dot_with_arrow(line, line_number, mapping, ast)
            line = self.rule_dereference_pointers(line, line_number)
            line = self.rule_custom_replacements(line, line_number)

        return line

    #########################
    #        RULES          #
    #########################
    def rule_replace_headers(self, line, line_number, mapping):
        """
        Changes the ros1 includes to corresponding ros2 includes and returns new line
        :param line: source line
        :param line_number: line number in src file
        :param mapping: ros1 to ros2 mapping for various types
        :return: str
        """
        try:
            header_mapping = mapping[AstConstants.INCLUSION_DIRECTIVE]
            for header in header_mapping:
                if header in line:
                    line = line.replace(header, CPPSourceCodePorter.get_ros2_name(header, header_mapping))
        except Exception as e:
            logging.warning("rule_replace_headers failed: " + self.file_path + ":", str(line_number))
            return line
        return line

    def rule_replace_namespace_ref(self, line, line_number, mapping):
        """
        Changes the ros1 namespace to corresponding ros2 namespaces and returns new line
        :param line: source line
        :param line_number: line number in src file
        :param mapping: ros1 to ros2 mapping for namespace_references
        :return: str
        """
        try:
            namespace_mapping = mapping[AstConstants.NAMESPACE_REF]
            for to_replace in namespace_mapping:
                replace_with = CPPSourceCodePorter.get_ros2_name(to_replace, namespace_mapping) + "::"
                to_replace += "::"
                line = Utilities.replace_word_in_line(line, to_replace, replace_with)
        except Exception as e:
            logging.warning("rule_replace_namespace_ref failed: " + self.file_path + ":", str(line_number))
            return line
        return line

    def rule_replace_call_expr(self, line, line_number, mapping, ast):
        """
        Changes the ros1 function calls to corresponding ros2 function calls  and returns new line
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: str
        """
        init_line = self.rule_init_call_found(line, line_number, mapping, ast)
        if init_line:
            return init_line
        else:
            try:
                call_expr_mapping = mapping[AstConstants.CALL_EXPR]
                if line_number in ast:
                    for token in ast[line_number][AstConstants.CALL_EXPR]:
                        fun_call = token[AstConstants.NAME]

                        if fun_call == Constants.VOID_CAST:
                            line = self.rule_handle_void_cast(line, line_number, ast, token)

                        if fun_call in call_expr_mapping and Constants.NODE_ARG_INFO in call_expr_mapping[fun_call]:
                            node_arg_info = call_expr_mapping[fun_call][Constants.NODE_ARG_INFO]
                            if Constants.NODE_ARG_REQ in node_arg_info and node_arg_info[Constants.NODE_ARG_REQ]:
                                if Constants.NODE_ARG_INDEX not in node_arg_info:
                                    raise Exception(Constants.NODE_ARG_INDEX + " key missing")
                                node_arg_ind = node_arg_info[Constants.NODE_ARG_INDEX]

                                line_tokens = token[AstConstants.LINE_TOKENS]

                                node_var_parameter = self.get_node_var_param(line_number, ast, True)

                                new_line_token = CPPSourceCodePorter.get_line_token_with_new_arg(line_tokens,
                                                                                                 node_arg_ind,
                                                                                                 node_var_parameter)
                                pattern = "(ros::)?" + fun_call + " *\("
                                replacement = "".join(new_line_token)
                                start_ind, end_ind = CPPSourceCodePorter.get_range_for_replacement(line, pattern)

                                line = line[:start_ind] + replacement + line[end_ind + 1:]
                                line = Utilities.replace_word_in_line(line,
                                                                      fun_call,
                                                                      CPPSourceCodePorter.get_ros2_name(
                                                                          fun_call, call_expr_mapping))
                            else:
                                line = Utilities.replace_word_in_line(line, fun_call,
                                                                      CPPSourceCodePorter.get_ros2_name(
                                                                          fun_call, call_expr_mapping))

                        # after replacement, if create_service<> in line, then do some special handling
                        line = self.rule_handle_create_service(line, line_number, mapping, ast, token)
                        line = self.rule_handle_create_subscription(line, line_number, mapping, ast, token)
            except Exception as e:
                logging.warning("rule_replace_call_expr failed: " + self.file_path + ":" + str(line_number))
                return line

            return line

    def rule_replace_var_decl(self, line, line_number, mapping, ast):
        """
        Changes the ros1 var declarations to corresponding ros2 var declarations and return new line
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: str
        """
        try:
            var_decl_mapping = mapping[AstConstants.VAR_DECL]
            if line_number in ast:
                for token in ast[line_number][AstConstants.VAR_DECL]:
                    var_type = token[AstConstants.VAR_TYPE]

                    template = None
                    if var_type in RosConstants.TEMPLATED_VARS:
                        template_type = self.template_from_call_back[var_type](token[AstConstants.NAME],
                                                                        var_type, line_number, mapping, ast)
                        if template_type == "":
                            template_type = Constants.UNKNOWN_TEMPLATE
                        template = "<" + template_type + ">"

                    # do not replace if used in template
                    pattern = "std::\w*\< *" + var_type + " *\>"
                    if re.search(pattern, line) is None:
                        if re.search("\< *" + var_type + " *\>", line):
                            continue

                    if var_type in line and var_type in var_decl_mapping:

                        # to_be_removed flag
                        to_be_removed = False
                        if Constants.TO_BE_REMOVED in var_decl_mapping[var_type]:
                            to_be_removed = var_decl_mapping[var_type][Constants.TO_BE_REMOVED]

                        if to_be_removed:
                            # comment out var to be removed
                            return "//" + line

                        try:
                            to_shared_ptr = var_decl_mapping[var_type][Constants.TO_SHARED_PTR]
                        except KeyError as e:
                            raise Exception(e.message + ": key not found")

                        ros2_name = CPPSourceCodePorter.get_ros2_name(var_type, var_decl_mapping)

                        if template:
                            ros2_name += template

                        if to_shared_ptr:
                            replace_with = "std::shared_ptr<" + ros2_name + ">"
                        else:
                            replace_with = ros2_name

                        line = Utilities.replace_word_in_line(line, var_type, replace_with)
                        return line
        except Exception as e:
            logging.warning("rule_replace_var_decl failed: " + self.file_path + ":" + str(line_number))
            return line
        return line

    def rule_replace_parm_decl(self, line, line_number, mapping, ast):
        """
        Replaces ROS1 parameter declaration in line to corresponding ROS2 parameter declaration
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: str
        """
        try:
            parm_decl_mapping = mapping[AstConstants.PARM_DECL]
            if line_number in ast:
                for token in ast[line_number][AstConstants.PARM_DECL]:
                    var_type = token[AstConstants.VAR_TYPE]
                    var_type = re.sub(r"^const *", "", var_type)
                    if var_type in parm_decl_mapping:
                        replace_with = CPPSourceCodePorter.get_ros2_name(var_type, parm_decl_mapping)
                        line = line.replace(var_type, replace_with)
        except Exception as e:
            logging.warning("rule_replace_parm_decl failed: " + self.file_path + ":" + str(line_number))
            return line
        return line

    def rule_replace_type_ref(self, line, line_number, mapping, ast):
        """
        Replaces ROS1 type references in line to corresponding ROS2 type references
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: str
        """
        try:
            type_ref_mapping = mapping[AstConstants.VAR_DECL]  # var_decl mapping will be used for type ref also
            if line_number in ast:
                for token in ast[line_number][AstConstants.TYPE_REF]:
                    var_type = token[AstConstants.VAR_TYPE]

                    if var_type not in type_ref_mapping:
                        var_type = token[AstConstants.LINE_TOKENS][0]

                    type_ref = var_type
                    # first try to find var type with namespace
                    if var_type in line:
                        replace_with = CPPSourceCodePorter.get_ros2_name(var_type, type_ref_mapping)
                    # try to find the var type without namespace
                    else:
                        replace_with = CPPSourceCodePorter.get_ros2_name(var_type, type_ref_mapping).split("::")[-1]
                        type_ref = var_type.split("::")[-1]

                    pattern = type_ref + " *>? *\("

                    # add node param
                    if type_ref_mapping[var_type][Constants.NODE_ARG_INFO][Constants.NODE_ARG_REQ]:
                        param_index = type_ref_mapping[var_type][Constants.NODE_ARG_INFO][Constants.NODE_ARG_INDEX]

                        param = self.get_node_var_param(line_number, ast, True)
                        line = self.add_param_at_index(line, pattern, param, param_index)

                    # adding any extra parameters specified
                    if AstConstants.TYPE_REF in Constants.EXTRA_PARAMS and \
                            type_ref in Constants.EXTRA_PARAMS[AstConstants.TYPE_REF]:
                        for extra_param, extra_param_index in Constants.EXTRA_PARAMS[AstConstants.TYPE_REF][type_ref]:
                            line = self.add_param_at_index(line, pattern, extra_param, extra_param_index)

                    if var_type in RosConstants.TEMPLATED_VARS:
                        replace_with += "<" + Constants.UNKNOWN_TEMPLATE + ">"
                    line = Utilities.replace_word_in_line(line, type_ref, replace_with)

        except Exception as e:
            logging.warning("rule_replace_type_ref failed: " + self.file_path + ":" + str(line_number) + str(e))
            return line
        return line

    def rule_replace_conversion_function(self, line, line_number, mapping, ast):
        """
        Replaces ROS1 conversion function in line to corresponding ROS2 conversion function
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: str
        """
        try:
            conversion_function_mapping = mapping[AstConstants.CONVERSION_FUNCTION]
            if line_number in ast:
                for token in ast[line_number][AstConstants.CONVERSION_FUNCTION]:
                    name = token[AstConstants.NAME]
                    if name in conversion_function_mapping:
                        line = line.replace(name, CPPSourceCodePorter.get_ros2_name(name, conversion_function_mapping))
        except Exception as e:
            logging.warning("rule_replace_conversion_function failed: " + self.file_path + ":" + str(line_number))
            return line
        return line

    def rule_replace_dot_with_arrow(self, line, line_number, mapping, ast):
        """
        Replaces all c++ '.' calls with '->' calls for pointer variables
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: str
        """
        try:
            var_decl_mapping = mapping[AstConstants.VAR_DECL]
            if line_number in ast:
                tokens = ast[line_number][AstConstants.DECL_REF_EXPR] + ast[line_number][AstConstants.MEMBER_REF_EXPR]
                for token in tokens:
                    var_name = token[AstConstants.NAME]
                    var_type = token[AstConstants.VAR_TYPE]

                    if var_type in var_decl_mapping and var_decl_mapping[var_type][Constants.TO_SHARED_PTR]:
                        pattern = "\\b" + var_name + "\."
                        replace_with = var_name + "->"
                        if re.search(pattern, line) is not None:
                            line = re.sub(pattern, replace_with, line)
        except Exception as e:
            logging.warning("rule_replace_dot_with_arrow failed: " + self.file_path + ":" + str(line_number))
            return line
        return line

    def rule_dereference_pointers(self, line, line_number):
        """
        Replaces all pointer calls with '->' calls for pointer variables
        :param line: source line
        :param line_number: line number in src file
        :return: str
        """

        # ToDO: write regex for finding pointers which are required to be deferenced
        return line

    def rule_replace_macros(self, line, line_number, mapping, ast):
        """
        Changes the ros1 macros to corresponding ros2 macros
        :param line: source line
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: ast in line by line format
        :return: str
        """
        try:
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
                                    arg_name = self.NODE_VAR_NAME + "->" + member_name + "()"
                                    new_line_token = CPPSourceCodePorter.get_line_token_with_new_arg(line_tokens,
                                                                                                     node_arg_ind,
                                                                                                     arg_name)
                                    pattern = macro + " *\("
                                    replacement = "".join(new_line_token)

                                    start_ind, end_ind = CPPSourceCodePorter.get_range_for_replacement(line, pattern)
                                    line = line[:start_ind] + replacement + line[end_ind + 1:]

                                line = Utilities.replace_word_in_line(line,
                                                                      macro,
                                                                      CPPSourceCodePorter.get_ros2_name(macro,
                                                                                                        macros_mapping))
                        else:
                            line = Utilities.replace_word_in_line(line,
                                                                  macro,
                                                                  CPPSourceCodePorter.get_ros2_name(macro,
                                                                                                    macros_mapping))
        except Exception as e:
            logging.warning("rule_replace_macros failed: " + self.file_path + ":" + str(line_number))
            return line
        return line

    def rule_handle_var_instantiation(self, line, line_number, mapping, ast):
        """
        Changes the ros1 var instantiation calls to corresponding ros2 calls
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :return: new source str
        """
        try:
            var_decl_mapping = mapping[AstConstants.VAR_DECL]
            if line_number in ast:
                for token in ast[line_number][AstConstants.CALL_EXPR]:
                    if token[AstConstants.VAR_TYPE] in var_decl_mapping and \
                            len(token[AstConstants.LINE_TOKENS]) > 0 and \
                            not CPPSourceCodePorter.is_var_already_instantiated(token, ast):

                        var_name = token[AstConstants.LINE_TOKENS][0]
                        pattern = "(?<=[ ,])" + var_name + "(?=[ ,;\)])"

                        if token[AstConstants.VAR_TYPE] == RosConstants.NODE_HANDLE:
                            replace_with = var_name + '(rclcpp::Node::make_shared(' + self.NODE_NAME + '))'
                            if len(token[AstConstants.LINE_TOKENS]) == 1:
                                line = re.sub(pattern, replace_with, line)
                            elif len(token[AstConstants.LINE_TOKENS]) > 1:
                                pattern = var_name + " *\("
                                start_ind, end_ind = CPPSourceCodePorter.get_range_for_replacement(line, pattern)
                                line = line[:start_ind] + replace_with + line[end_ind + 1:]
                            return line
                        elif token[AstConstants.VAR_TYPE] == RosConstants.DURATION:
                            replace_with = CPPSourceCodePorter.get_ros2_name(token[AstConstants.VAR_TYPE], var_decl_mapping)
                            template_var = token[AstConstants.LINE_TOKENS][-2]
                            template_type = Constants.UNKNOWN_TEMPLATE
                            for tem_candidate in ast[line_number][AstConstants.DECL_REF_EXPR]:
                                if tem_candidate[AstConstants.NAME] == template_var:
                                    template_type = tem_candidate[AstConstants.VAR_TYPE]
                            line = line.replace(RosConstants.DURATION, replace_with + "<" + template_type + ">")
                        elif len(token[AstConstants.LINE_TOKENS]) == 1:
                            ros2_type = CPPSourceCodePorter.get_ros2_name(token[AstConstants.VAR_TYPE], var_decl_mapping)
                            replace_with = var_name + '(std::make_shared<' + ros2_type + '>())'
                            line = re.sub(pattern, replace_with, line)

        except Exception as e:
            logging.warning("rule_handle_var_instantiation failed: " + self.file_path + ":" + str(line_number))
            return line
        return line

    def rule_replace_removed_var(self, line, line_number, mapping, ast):
        """
        Removes removes usage of removed var_type variables
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: line by line ast
        :return: str
        """
        try:
            var_decl_mapping = mapping[AstConstants.VAR_DECL]
            if line_number in ast:
                tokens = ast[line_number][AstConstants.DECL_REF_EXPR]
                for token in tokens:
                    var_name = token[AstConstants.NAME]
                    var_type = token[AstConstants.VAR_TYPE]

                    if var_type in var_decl_mapping and var_decl_mapping[var_type][Constants.TO_BE_REMOVED]:
                        pattern = "\\b" + var_name + "\\b"
                        replace_with = ""
                        if re.search(pattern, line) is not None:
                            # first remove the var_name usage
                            line = re.sub(pattern, replace_with, line)

                            # now if after var name removal there is an extra `,` remove it
                            line = CPPSourceCodePorter.remove_extra_comma(line)
        except Exception as e:
            logging.warning("rule_replace_removed_var failed: " + self.file_path + ":" + str(line_number))
            return line
        return line

    def rule_replace_function_signature(self, line, line_number, mapping, ast):
        """
        Modifies function signatures if required
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for various types
        :param ast: line by line ast
        :return: str
        """
        try:
            if line_number in ast and AstConstants:
                decl_list = ast[line_number][AstConstants.CXX_METHOD] + ast[line_number][AstConstants.FUNCTION_DECL]
                for token in decl_list:
                    function_signature = token[AstConstants.VAR_TYPE]

                    for var_type in self.vars_to_be_removed:
                        if var_type in function_signature:
                            line = CPPSourceCodePorter.remove_param_type_from_line(line, var_type)
        except Exception as e:
            logging.warning("rule_replace_function_signature failed: " + self.file_path + ":" + str(line_number))
            return line
        return line

    def rule_custom_replacements(self, line, line_number):
        """
        Does some custom replacements in line, if not done by other rules
        :param line: source code line
        :param line_number: line number in src file
        :return: str
        """
        try:
            # if line still contains "::ConstPtr" replace it with, "::SharedPtr"
            line = Utilities.replace_word_in_line(line, "::ConstPtr", "::SharedPtr")

            # remove & after ::SharedPtr
            line = re.sub("::SharedPtr *&", "::SharedPtr", line)
        except Exception as e:
            logging.warning("rule_custom_replacements failed: " + self.file_path + ":" + str(line_number))
            return line

        return line

    #########################
    #      SUB-RULES        #
    #########################
    def rule_handle_void_cast(self, line, line_number, ast, token):
        """
        Removes static_cast<void *> pattern from line for valid candidates and return new line
        :param line: source code line
        :param line_number: line number of code in the file
        :param ast: line by line ast
        :param token: token attributes dict
        :return: str
        """
        try:
            var_name = token[AstConstants.LINE_TOKENS][0]
            pattern = "static_cast *< *void *\* *> *\( *" + var_name + " *\)"

            if AstConstants.MEMBER_REF_EXPR in ast[line_number]:
                for member_ref_token in ast[line_number][AstConstants.MEMBER_REF_EXPR]:
                    if member_ref_token[AstConstants.NAME] == var_name:
                        var_type = member_ref_token[AstConstants.VAR_TYPE]
                        if var_type in Constants.REMOVE_VOID_POINTER_CAST:
                            line = re.sub(pattern, var_name, line)
                            return line
        except Exception as e:
            logging.warning("rule_handle_void_cast failed: " + self.file_path + ":" + str(line_number))
            return line
        return line

    def rule_handle_create_service(self, line, line_number, mapping, ast, token):
        """
        Handles create_service call, adds lambda wrapper for new call_back and adds template argument and returns new
        line
        :param line: souce code line
        :param line_number: line number in source code
        :param mapping: ROS1 to ROS2 mapping dict
        :param ast: line by line ast
        :param token: token attributes dict
        :return: str
        """
        try:
            if RosConstants.CREATE_SERVICE in line:

                line_tokens = token[AstConstants.LINE_TOKENS]
                last_comma_index = len(line_tokens) - 1 - line_tokens[::-1].index(',')
                call_back_name = line_tokens[last_comma_index - 1]

                arg_list = self.get_arguments_for_function(call_back_name)
                lambda_wrap = '[this](std::shared_ptr<rmw_request_id_t> request_header'
                call_back_call = call_back_name + "("
                for ind, arg in enumerate(arg_list):
                    arg_var = "arg" + str(ind)
                    lambda_wrap += ",std::shared_ptr<" \
                                   + CPPSourceCodePorter.get_ros2_name(arg, mapping[AstConstants.VAR_DECL]) \
                                   + "> " + arg_var
                    call_back_call += arg_var
                    if ind != len(arg_list) - 1:
                        call_back_call += ","

                call_back_call += ")"
                lambda_wrap += ") { return " + call_back_call + ";}"

                first_comma_index = line_tokens.index(',')
                service_name = line_tokens[first_comma_index - 1]

                # remove parameters after create_service<>(a, b, c) => create_service<>
                create_service_args = '(' + service_name + ',' + lambda_wrap + ")"
                line = re.sub(RosConstants.CREATE_SERVICE + "\(.*\)", RosConstants.CREATE_SERVICE +
                              create_service_args, line)

                replace_with = RosConstants.CREATE_SERVICE[:-1] + \
                               self.get_ros_service_template(token[AstConstants.NAME],
                                                             token[AstConstants.VAR_TYPE],
                                                             line_number, mapping, ast) + ">"
                line = line.replace(RosConstants.CREATE_SERVICE, replace_with)
        except Exception as e:
            logging.warning("rule_handle_create_service failed: " + self.file_path + ":" + str(line_number))
            return line

        return line

    def rule_handle_create_subscription(self, line, line_number, mapping, ast, token):
        """
        Handles create_subscription call, adds template argument and returns new line
        :param line: souce code line
        :param line_number: line number in source code
        :param mapping: ROS1 to ROS2 mapping dict
        :param ast: line by line ast
        :param token: token attributes dict
        :return: str
        """
        try:
            if RosConstants.CREATE_SUBSCRIPTION in line:

                line_tokens = token[AstConstants.LINE_TOKENS]
                last_paren_index = len(line_tokens) - 1 - line_tokens[::-1].index(')')
                call_back_name = line_tokens[last_paren_index - 1]

                ros2_template = self.get_ros2type_for_ros1_argument_type(call_back_name, mapping)
                # remove SharedPtr
                ros2_template = "::".join(ros2_template.split("::")[:-1])
                ros2_template = self.get_ros2_name(ros2_template, mapping[AstConstants.VAR_DECL])

                replace_with = RosConstants.CREATE_SUBSCRIPTION + "<" + ros2_template + ">"
                line = line.replace(RosConstants.CREATE_SUBSCRIPTION, replace_with)
        except Exception as e:
            logging.warning("rule_handle_create_subscription failed: " + self.file_path + ":" + str(line_number))
            return line

        return line

    def rule_init_call_found(self, line, line_number, mapping, ast):
        """
        If line contains ros::init call, it replace the call with ros2 init call and returns new line, otherwise returns
        None
        :param line: line to convert
        :param line_number: line number of line in source code
        :param mapping: ros1 to ros2 mapping for call_expr
        :param ast: ast in line by line format
        :return: str or None
        """
        try:
            if line_number in ast and RosConstants.INIT_PATTERN in line:
                token = CPPSourceCodePorter.find_token_in_ast(AstConstants.NAME, RosConstants.INIT_CALL_EXPR,
                                                              ast[line_number][AstConstants.CALL_EXPR])
                if token:
                    line_tokens = token[AstConstants.LINE_TOKENS]
                    # tokens for init:  ['ros', '::', 'init', '(', 'argc', ',', 'argv', ',', '"talker"', ')']
                    # so min length should be 10
                    if len(line_tokens) >= RosConstants.ROS_INIT_LINE_TOKEN_LENGTH:

                        ros2_name = CPPSourceCodePorter.get_ros2_name(line_tokens[2], mapping)

                        init_pattern = RosConstants.INIT_PATTERN + ".*\)"
                        replace_with = line_tokens[0] + line_tokens[1] + ros2_name + \
                            line_tokens[3] + line_tokens[4] + line_tokens[5] + line_tokens[6] + ")"
                        return re.sub(init_pattern, replace_with, line)
                    else:
                        logging.warning("line tokens for ros::init invalid: " + self.file_path + ":", str(line_number))
                        return line
        except Exception as e:
            logging.warning("rule_init_call_found failed: " + self.file_path + ":", str(line_number))
            return line
        return None

    #########################
    #        HELPERS        #
    #########################
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
        :return: dict or None
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
    def get_range_for_replacement(line, pattern):
        """
        Returns start and end index for the pattern of type `fun_name(`, end index would be the closing parenthesis for
        `fun_name`
        :param line: source code line containing `fun_name`
        :param pattern: function pattern to look for
        :return: tuple
        """
        found_result = re.search(pattern, line)
        if found_result is None:
            raise Exception("Pattern `" + pattern + "` not found in: " + line)
        start_ind = found_result.start()
        end_ind = start_ind

        i = start_ind
        open_cnt = 0
        first_bracket_found = False
        while i < len(line):
            if line[i] == '(':
                first_bracket_found = True
                open_cnt += 1
            elif line[i] == ')':
                open_cnt -= 1

            if first_bracket_found and open_cnt == 0:
                end_ind = i
                break

            i += 1

        return start_ind, end_ind

    @staticmethod
    def is_var_already_instantiated(token, ast):
        """
        Checks if a var_type has already been instantiated
        :param token: token dict
        :param ast: line by line ast
        :return: boolean
        """
        name = token[AstConstants.NAME]

        # if instantiated using some member function
        for token in ast[token[AstConstants.LINE]][AstConstants.MEMBER_REF_EXPR]:
            if token[AstConstants.NAME] == name:
                return True

        return False

    @staticmethod
    def add_param_at_index(line, pattern, param, param_index):
        """
        Adds `param` as an argument at index `param_index` for call starting with `pattern` in `line` and returns the
        new line
        :param line: source code line
        :param pattern: regex pattern to match
        :param param: parameter name
        :param param_index: index for `param`, 0-based
        :return: str
        """

        pattern_search = re.search(pattern, line)
        if pattern_search is None:
            return line

        open_parenthesis_ind = pattern_search.span()[1] - 1

        pattern_start, close_parenthesis_ind = \
            CPPSourceCodePorter.get_range_for_replacement(line, pattern)

        # parameter start just after opening parenthesis
        param_start = open_parenthesis_ind + 1

        param_list = []
        stack = []

        for ind, ch in enumerate(line[open_parenthesis_ind + 1:close_parenthesis_ind + 1]):
            ind += open_parenthesis_ind + 1

            # this check is to avoid condition like fun(a, b, f(), c)
            if len(stack) == 0:
                if ch == ",":
                    param_list.append(line[param_start: ind].strip())
                    param_start = ind + 1
                if ch == ")":
                    arg = line[param_start: ind].strip()
                    if arg != "":
                        param_list.append(arg)
                    break
            if ch == "(":
                stack.append(ch)
            elif ch == ")":
                stack.pop()

        # insert the new param at `param_index`
        param_list.insert(param_index, param)

        line = line[:open_parenthesis_ind + 1] + ",".join(param_list) + line[close_parenthesis_ind:]
        return line

    @staticmethod
    def remove_extra_comma(line):
        """
        Removed extra comma from line.
        :param line: source code line
        :return: str
        """

        # remove if pattern like fun(a, b, , c)
        line = re.sub(r", * ,", ",", line)

        # remove if pattern like fun( , a, b)
        line = re.sub(r"\( *, *", "(", line)

        # remove if pattern like fun(a, b, )
        line = re.sub(r", *\)", ")", line)

        return line

    @staticmethod
    def remove_param_type_from_line(line, param_type):
        """
        Removes `param_type` from line if mentioned as a function parameter
        :param line: source code line
        :param param_type: variable type
        :return: str
        """
        var_pattern = " *(\\w*)? *" + param_type + " *([\*, &](\\w*)?)? *"

        while re.search(var_pattern, line):
            # remove param_type as only param, fun(param_type)
            pattern = "\(" + var_pattern + "\)"
            line = re.sub(pattern, "()", line)

            # remove param_type as first_param, fun(param_type, b, c)
            pattern = "\(" + var_pattern + ","
            line = re.sub(pattern, "(", line)

            # remove param_type as middle param, fun(a,b,c)
            pattern = "," + var_pattern + ","
            line = re.sub(pattern, ",", line)

            # remove param_type as last param, fun(a,b,c)
            pattern = "," + var_pattern + "\)"
            line = re.sub(pattern, ")", line)

        return line

    def get_ros2type_for_ros1_argument_type(self, call_back_name, mapping):
        """
        Get the first argument type from the function signature of call_back_name and then return the corresponding
        ros2 type. Return "" if not found
        :param call_back_name: callback function name
        :param mapping: ros1 to ros2 mapping for various types
        :return: str
        """
        arg_type = ""
        if AstConstants.DECL_REF_EXPR in self.global_ast:
            for token in self.global_ast[AstConstants.DECL_REF_EXPR]:
                var_type = token[AstConstants.VAR_TYPE]
                if token[AstConstants.NAME] == call_back_name and "(" in var_type and ")" in var_type:
                    args = self.get_arguments_for_function(call_back_name)
                    if len(args) > 0 and args[0] in mapping[AstConstants.VAR_DECL]:
                        return CPPSourceCodePorter.get_ros2_name(args[0], mapping[AstConstants.VAR_DECL])

        return arg_type

    def get_ros_service_template(self, var_name, var_type, line_number, mapping, ast):
        """
        Returns the ros::ServiceServer var declaration or instantiation template argument for ros2 type, by finding the
        type from the ast
        :param var_name: name of the variable
        :param var_type: type of the variable
        :param mapping: ros1 to ros2 mapping for various types
        :return: str
        """
        ros2_arg_type = ""
        if AstConstants.CALL_EXPR in self.global_ast:
            for token in self.global_ast[AstConstants.CALL_EXPR]:
                line_tokens = token[AstConstants.LINE_TOKENS]
                if token[AstConstants.VAR_TYPE] == var_type and \
                        var_name in line_tokens:
                    last_comma_index = len(line_tokens) - 1 - line_tokens[::-1].index(',')
                    call_back_name = line_tokens[last_comma_index - 1]

                    # mapping for "AudioTextConversationRequest" is
                    # "lex_common_msgs::srv::AudioTextConversation::Request, but template argument would be only
                    # "lex_common_msgs::srv::AudioTextConversation
                    ros2_arg_type = self.get_ros2type_for_ros1_argument_type(call_back_name, mapping)
                    ros2_arg_type = "::".join(ros2_arg_type.split("::")[:-1])

                    return ros2_arg_type

        return ros2_arg_type

    def get_ros_subscribe_template(self, var_name, var_type, line_number, mapping, ast):
        """
        Returns the ros::ServiceServer var declaration or instantiation template argument for ros2 type, by finding the
        type from the ast
        :param var_name: name of the variable
        :param var_type: type of the variable
        :param mapping: ros1 to ros2 mapping for various types
        :return: str
        """
        ros2_arg_type = ""
        if line_number in ast:
            for token in ast[line_number][AstConstants.VAR_DECL]:
                line_tokens = token[AstConstants.LINE_TOKENS]
                if token[AstConstants.VAR_TYPE] == var_type and \
                        var_name == token[AstConstants.NAME] and \
                        RosConstants.TEMPLATED_VARS[var_type] in line_tokens:
                    last_parenthesis_index = len(line_tokens) - 1 - line_tokens[::-1].index(')')
                    call_back_name = line_tokens[last_parenthesis_index - 1]
                    ros2_arg_type = self.get_ros2type_for_ros1_argument_type(call_back_name, mapping)

                    # remove ::SharedPtr from end
                    ros2_arg_type = "::".join(ros2_arg_type.split("::")[:-1])
                    return ros2_arg_type

        return ros2_arg_type

    def get_node_var_param_for_unit_test(self, line_number, ast):
        """
        Searches for declaration of NodeHandle var before the line_number
        :param line_number: line number in source
        :param ast: line by line ast
        :return: str or None
        """
        while line_number >= 0:
            if line_number in ast:
                for token in ast[line_number][AstConstants.VAR_DECL]:
                    if token[AstConstants.VAR_TYPE] == RosConstants.NODE_HANDLE:
                        return token[AstConstants.NAME]
            line_number -= 1
        return None

    def get_node_var_param(self, line_number, ast, static_cast=False):
        """
        Finds the instantiation of class containing `self.NODE_VAR_PARENT_CLASS` and return the var_name. Optionally, it
        will be statically casted to `std::shared_ptr<rclcpp::Node>&` depending on `static_cast`
        :param line_number: line_number where node_var_param is to be added
        :param ast: line by line ast
        :param static_cast: flag to check if var_name needs static_cast
        :return: str
        """
        if self.is_porting_unit_test:
            node_var = self.get_node_var_param_for_unit_test(line_number, ast)
            return "" if node_var is None else node_var

        if self.NODE_VAR_PARENT_CLASS is None:
            return self.NODE_VAR_NAME
        var_name = None
        while line_number >= 0:
            if line_number in ast:
                for token in ast[line_number][AstConstants.VAR_DECL]:
                    if self.NODE_VAR_PARENT_CLASS in token[AstConstants.VAR_TYPE]:
                        var_name = token[AstConstants.NAME]
                        break
                if var_name:
                    if static_cast:
                        var_name = "static_cast<std::shared_ptr<rclcpp::Node>&>(" + var_name + ")"
                    break
            line_number -= 1

        if var_name is None:
            return self.NODE_VAR_NAME

        return var_name

    def get_arguments_for_function(self, function_name):
        """
        Given `function_name`, returns list of arguments for it
        :param function_name: name of the function
        :return: list
        """
        function_args = []
        if AstConstants.DECL_REF_EXPR in self.global_ast:
            for token in self.global_ast[AstConstants.DECL_REF_EXPR]:
                if token[AstConstants.NAME] == function_name:
                    # signature will be like "bool (lex_common_msgs::AudioTextConversationRequest &, lex_common_msgs::AudioTextConversationResponse &)"
                    function_signature = token[AstConstants.VAR_TYPE]

                    arg_str = re.search("\(.*\)", function_signature)
                    if arg_str:
                        raw_arg_list = arg_str.group(0)[1:-1].split(",")
                        for arg in raw_arg_list:

                            # removing "const", "static" etc from beginning
                            for qualifier in self.filter_out[AstConstants.TYPE_QUALIFIER]:
                                pattern = qualifier + " "
                                if arg.startswith(pattern):
                                    arg = arg[len(pattern):]

                            arg = re.sub(r" *[&,*]", "", arg)
                            function_args.append(arg.strip())
                        return function_args
        return function_args

