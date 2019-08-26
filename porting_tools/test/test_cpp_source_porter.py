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


import os
import unittest

from Constants import AstConstants, Constants
from porting_tools.cpp_source_code_porter import CPPSourceCodePorter
from utilities import Utilities


class TestCPPSourceCodePorterTalker(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mapping = Utilities.get_consolidated_mapping()

        ast_dict = {
            AstConstants.NON_UNIT_TEST: Utilities.read_json_file(os.path.join("porting_tools", "test", "ast_dump_talker_cpp.json"))
        }
        cls.ast_line_wise = Utilities.store_ast_line_by_line(ast_dict[AstConstants.NON_UNIT_TEST])

        # file_path as mentioned in the `ast_dump_talker_cpp.json` file, will not be used for file reading purpose
        # as the path is system specific
        cls.file_path_in_ast = "src/talker/talker.cpp"
        cls.cpp_porter = CPPSourceCodePorter(ast_dict, cls.file_path_in_ast)

    @classmethod
    def tearDownClass(cls):
        pass

    def test_rule_replace_headers(self):
        source = r'#include std_msgs/String.h'
        result = r'#include std_msgs/msg/string.hpp'
        self.assertEqual(self.cpp_porter.rule_replace_headers(source, 3, self.mapping), result)

    def test_rule_replace_namespace(self):
        source = r'std_msgs::String msg;'
        result = r'std_msgs::msg::String msg;'
        self.assertEqual(self.cpp_porter.rule_replace_namespace_ref(source, 16, self.mapping), result)

    def test_rule_init_call_found_failure(self):
        source = r'ros::different_init(argc, argv, "talker")'
        result = None
        self.assertEqual(self.cpp_porter.rule_init_call_found(source, 6, self.mapping,
                                                              self.ast_line_wise[self.file_path_in_ast]), result)

    def test_rule_init_call_found_success(self):
        source = r'ros::init(argc, argv, "talker")'
        result = r'ros::init(argc,argv)'
        self.assertEqual(self.cpp_porter.rule_init_call_found(source, 6, self.mapping,
                                                              self.ast_line_wise[self.file_path_in_ast]), result)

    def test_rule_replace_call_expr_with_node_arg(self):
        source = r'ros::spinOnce()'
        result = r'ros::spin_some(node)'
        self.assertEqual(self.cpp_porter.rule_replace_call_expr(source, 29, self.mapping,
                                                                self.ast_line_wise[self.file_path_in_ast]),
                         result)

    def test_rule_replace_call_expr_without_node_arg(self):
        source = r'ros::Publisher chatter_pub = node.advertise<std_msgs::String>("chatter", 1000);'
        result = r'ros::Publisher chatter_pub = node.create_publisher<std_msgs::String>("chatter", 1000);'
        self.assertEqual(self.cpp_porter.rule_replace_call_expr(source, 10, self.mapping,
                                                                self.ast_line_wise[self.file_path_in_ast]),
                         result)

    def test_rule_replace_var_decl(self):
        source = r'ros::NodeHandle node;'
        result = r'std::shared_ptr<ros::Node> node;'
        self.assertEqual(self.cpp_porter.rule_replace_var_decl(source, 8, self.mapping,
                                                               self.ast_line_wise[self.file_path_in_ast]), result)

    def test_rule_handle_var_instantiation_with_equal_to(self):
        # source and result will be same as `chatter_pub` has already been instantiated
        source = r'ros::Publisher chatter_pub = node.advertise<std_msgs::String>("chatter", 1000);'
        result = r'ros::Publisher chatter_pub = node.advertise<std_msgs::String>("chatter", 1000);'
        self.assertEqual(self.cpp_porter.rule_handle_var_instantiation(source, 10,
                                                                       self.mapping,
                                                                       self.ast_line_wise[self.file_path_in_ast]), result)

    def test_rule_handle_var_instantiation_without_equal_to(self):
        source = r'std_msgs::String msg;'
        result = r'std_msgs::String msg(std::make_shared<std_msgs::String>());'
        self.assertEqual(self.cpp_porter.rule_handle_var_instantiation(source, 16, self.mapping,
                                                                       self.ast_line_wise[self.file_path_in_ast]), result)

    def test_rule_replace_dot_with_arrow(self):
        source = r'chatter_pub.publish(msg);'
        result = r'chatter_pub->publish(msg);'
        self.assertEqual(self.cpp_porter.rule_replace_dot_with_arrow(source, 27, self.mapping,
                                                                     self.ast_line_wise[self.file_path_in_ast]),
                         result)

    def test_rule_replace_macros_with_node_arg(self):
        source = r'ROS_INFO("%s\n",msg.data.c_str());'
        result = r'RCLCPP_INFO(node->get_logger(),"%s\n",msg.data.c_str());'
        self.assertEqual(self.cpp_porter.rule_replace_macros(source, 25, self.mapping,
                                                             self.ast_line_wise[self.file_path_in_ast]), result)

# file paths as mentioned in the `ast_dump_lex_node.json` file, will not be used for file reading purpose
# as the path is system specific
LEX_NODE_FILE_LIST = [
    u'src/main.cpp',
    u'src/lex_node.cpp',
    u'include/lex_node/lex_node.h',
    u'src/lex_param_helper.cpp',
    u'include/lex_node/lex_configuration.h',
    u'include/lex_node/lex_param_helper.h'
]


class TestCPPSourceCodePorterLexNodeMainCPP(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mapping = Utilities.get_consolidated_mapping()

        ast_dict = {
            AstConstants.NON_UNIT_TEST: Utilities.read_json_file(os.path.join("porting_tools", "test", "ast_dump_lex_node.json"))
        }
        cls.ast_line_wise = Utilities.store_ast_line_by_line(ast_dict[AstConstants.NON_UNIT_TEST])

        cls.file_path_in_ast = LEX_NODE_FILE_LIST[0]
        cls.cpp_porter = CPPSourceCodePorter(ast_dict, cls.file_path_in_ast)

    @classmethod
    def tearDownClass(cls):
        pass

    def test_rule_replace_type_ref_with_no_extra_param(self):
        source = 'Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("lex_node")'
        result = 'Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("lex_node",Aws::Utils::Logging::LogLevel::Debug,static_cast<std::shared_ptr<rclcpp::Node>&>(lex_node))'
        self.assertEqual(self.cpp_porter.rule_replace_type_ref(source, 38,
                                                               self.mapping,
                                                               self.ast_line_wise[self.file_path_in_ast]), result)

    def test_rule_replace_type_ref_with_extra_param(self):
        source = 'Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("lex_node")'
        result = 'Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("lex_node",Aws::Utils::Logging::LogLevel::Debug,static_cast<std::shared_ptr<rclcpp::Node>&>(lex_node))'
        self.assertEqual(self.cpp_porter.rule_replace_type_ref(source, 38,
                                                               self.mapping,
                                                               self.ast_line_wise[self.file_path_in_ast]), result)


class TestCPPSourceCodePorterLexNodeLexNodeCPP(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mapping = Utilities.get_consolidated_mapping()

        ast_dict = {
            AstConstants.NON_UNIT_TEST: Utilities.read_json_file(os.path.join("porting_tools", "test", "ast_dump_lex_node.json"))
        }
        cls.ast_line_wise = Utilities.store_ast_line_by_line(ast_dict[AstConstants.NON_UNIT_TEST])

        cls.file_path_in_ast = LEX_NODE_FILE_LIST[1]
        cls.cpp_porter = CPPSourceCodePorter(ast_dict, cls.file_path_in_ast)

    @classmethod
    def tearDownClass(cls):
        pass

    def test_rule_replace_type_ref_with_no_extra_param(self):
        source = 'params = std::make_shared<Client::Ros1NodeParameterReader>();'
        result = 'params = std::make_shared<Client::Ros2NodeParameterReader>(static_cast<std::shared_ptr<rclcpp::Node>&>(lex_node));'
        self.assertEqual(self.cpp_porter.rule_replace_type_ref(source, 172,
                                                               self.mapping,
                                                               self.ast_line_wise[self.file_path_in_ast]), result)

    def test_rule_type_ref_in_function_parameter(self):
        source = 'bool LexNode::LexServerCallback(lex_common_msgs::AudioTextConversationRequest & request,\n' \
                 'lex_common_msgs::AudioTextConversationResponse & response)'

        result = 'bool LexNode::LexServerCallback(std::shared_ptr<lex_common_msgs::srv::AudioTextConversation::Request> request,\n' \
                 'lex_common_msgs::AudioTextConversationResponse & response)'

        self.assertEqual(self.cpp_porter.rule_replace_parm_decl(source, 199,
                                                                self.mapping,
                                                                self.ast_line_wise[self.file_path_in_ast]), result)

    def test_rule_replace_call_expr_advertise_service(self):
        source = 'node_handle_.advertiseService<>("lex_conversation", &LexNode::LexServerCallback, this);'
        result = 'node_handle_.create_service<lex_common_msgs::srv::AudioTextConversation>("lex_conversation",[this](std::shared_ptr<rmw_request_id_t> request_header,' \
                 'std::shared_ptr<lex_common_msgs::srv::AudioTextConversation::Request> arg0,' \
                 'std::shared_ptr<lex_common_msgs::srv::AudioTextConversation::Response> arg1) { return LexServerCallback(arg0,arg1);});'

        self.assertEqual(self.cpp_porter.rule_replace_call_expr(source, 188,
                                                                self.mapping,
                                                                self.ast_line_wise[self.file_path_in_ast]), result)


class TestCPPSourceCodePorterLexNodeLexNodeH(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mapping = Utilities.get_consolidated_mapping()

        ast_dict = {
            AstConstants.NON_UNIT_TEST: Utilities.read_json_file(os.path.join("porting_tools", "test", "ast_dump_lex_node.json"))
        }
        cls.ast_line_wise = Utilities.store_ast_line_by_line(ast_dict[AstConstants.NON_UNIT_TEST])

        cls.file_path_in_ast = LEX_NODE_FILE_LIST[2]
        cls.cpp_porter = CPPSourceCodePorter(ast_dict, cls.file_path_in_ast)

    @classmethod
    def tearDownClass(cls):
        pass

    def test_rule_replace_conversion_function(self):
        source = 'explicit operator ros::NodeHandle &() { return node_handle_; }'
        result = 'explicit operator std::shared_ptr<ros::Node> &() { return node_handle_; }'
        self.assertEqual(self.cpp_porter.rule_replace_conversion_function(source, 128,
                                                                          self.mapping,
                                                                          self.ast_line_wise[self.file_path_in_ast]), result)

    def test_rule_replace_call_expr_with_void_cast(self):
        source = 'bool IsServiceValid() { return (nullptr != static_cast<void *>(lex_server_)); }'
        result = 'bool IsServiceValid() { return (nullptr != lex_server_); }'
        self.assertEqual(self.cpp_porter.rule_replace_call_expr(source, 90,
                                                                self.mapping,
                                                                self.ast_line_wise[self.file_path_in_ast]), result)


class TestCPPSourceCodePorterHelpers(unittest.TestCase):
    def test_get_line_token_with_new_arg_0(self):
        source = ["ros", "::", "spinOnce", "(", "param1", ")"]
        result = ["ros", "::", "spinOnce", "(", "param0", ",", "param1", ")"]
        self.assertEqual(CPPSourceCodePorter.get_line_token_with_new_arg(source, 0, "param0"), result)

    def test_get_line_token_with_new_arg_1(self):
        source = ["ros", "::", "spinOnce", "(", ")"]
        result = ["ros", "::", "spinOnce", "(", "param0", ")"]
        self.assertEqual(CPPSourceCodePorter.get_line_token_with_new_arg(source, 0, "param0"), result)

    def test_get_range_for_replacement_with_another_funcall_param(self):
        source = 'outer_function(a, b, c, fun(g, h), d, e)'
        result = (0, 39)
        self.assertEqual(CPPSourceCodePorter.get_range_for_replacement(source, "outer_function\("), result)

    def test_get_range_for_replacement_without_another_funcall_param(self):
        source = 'outer_function(a, b, c, fun(g, h), d, e)'
        result = (24, 32)
        self.assertEqual(CPPSourceCodePorter.get_range_for_replacement(source, "fun\("), result)

    def test_add_param_at_index_test0(self):
        source = 'outer_function(a, b, c, fun(), d, e)'
        result = 'outer_function(a, b, c, fun(g0), d, e)'
        self.assertEqual(CPPSourceCodePorter.add_param_at_index(source, "fun\(", 'g0', 0), result)

    def test_add_param_at_index_test1(self):
        source = 'outer_function(a, b, c, fun(g, h), d, e)'
        result = 'outer_function(a,b,c,c1,fun(g, h),d,e)'
        self.assertEqual(CPPSourceCodePorter.add_param_at_index(source, "outer_function\(", 'c1', 3), result)

    def test_add_param_at_index_test2(self):
        source = 'outer_function<template>(a, b, c, fun(g, h), d, e)'
        result = 'outer_function<template>(a,b,c,c1,fun(g, h),d,e)'

        pattern = 'template' + " *>? *\("
        self.assertEqual(CPPSourceCodePorter.add_param_at_index(source, pattern, 'c1', 3), result)


if __name__ == '__main__':
    unittest.main()
