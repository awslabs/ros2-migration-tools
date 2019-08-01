src = '#include <sstream>\
#include "ros/ros.h"\
#include "std_msgs/String.h"\
int main(int argc, char **argv)\
{\
  ros::init(argc, argv, "talker");\
\
  ros::NodeHandle node;\
\
  ros::Publisher chatter_pub = node.advertise<std_msgs::String>("chatter", 1000);\
\
  ros::Rate loop_rate(10);\
\
  int count = 0;\
\
  std_msgs::String msg;\
\
  while (ros::ok())\
  {\
    std::stringstream ss;\
    ss << "hello world " << count++;\
\
    msg.data = ss.str();\
\
    ROS_INFO("%s", msg.data.c_str());\
\
    chatter_pub.publish(msg);\
\
    ros::spinOnce();\
\
    loop_rate.sleep();\
  }\
  return 0;\
}'

import os
import unittest

from Constants import AstConstants, Constants
from porting_tools.cpp_source_code_porter import CPPSourceCodePorter
from utilities import Utilities


class TestCPPSourceCodePorterTalker(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mapping = Utilities.read_json_file(Constants.MAPPING_FILE_NAME)

        ast_dict = Utilities.read_json_file(os.path.join("porting_tools", "test", "ast_dump_talker_cpp.json"))
        cls.ast_line_wise = Utilities.get_line_by_line_ast(ast_dict)

        cls.cpp_porter = CPPSourceCodePorter(ast_dict, "unit_test")

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
        self.assertEqual(self.cpp_porter.rule_init_call_found(source, 6, self.mapping, self.ast_line_wise), result)

    def test_rule_init_call_found_success(self):
        source = r'ros::init(argc, argv, "talker")'
        result = r'ros::init(argc,argv)'
        self.assertEqual(self.cpp_porter.rule_init_call_found(source, 6, self.mapping, self.ast_line_wise), result)

    def test_rule_replace_call_expr_with_node_arg(self):
        source = r'ros::spinOnce()'
        result = r'ros::spin_some(node)'
        self.assertEqual(self.cpp_porter.rule_replace_call_expr(source, 29, self.mapping, self.ast_line_wise),
                         result)

    def test_rule_replace_call_expr_without_node_arg(self):
        source = r'ros::Publisher chatter_pub = node.advertise<std_msgs::String>("chatter", 1000);'
        result = r'ros::Publisher chatter_pub = node.create_publisher<std_msgs::String>("chatter", 1000);'
        self.assertEqual(self.cpp_porter.rule_replace_call_expr(source, 10, self.mapping, self.ast_line_wise),
                         result)

    def test_rule_replace_var_decl(self):
        source = r'ros::NodeHandle node;'
        result = r'std::shared_ptr<ros::Node> node;'
        self.assertEqual(self.cpp_porter.rule_replace_var_decl(source, 8, self.mapping, self.ast_line_wise), result)

    def test_rule_handle_var_instantiation_with_equal_to(self):
        source = r'ros::Publisher chatter_pub = node.advertise<std_msgs::String>("chatter", 1000);'
        result = r'ros::Publisher chatter_pub = node.advertise<std_msgs::String>("chatter", 1000);'
        self.assertEqual(self.cpp_porter.rule_handle_var_instantiation(source, 10, self.mapping,
                                                                       self.ast_line_wise), result)

    def test_rule_handle_var_instantiation_without_equal_to(self):
        source = r'std_msgs::String msg;'
        result = r'std_msgs::String msg(std::make_shared<std_msgs::String>());'
        self.assertEqual(self.cpp_porter.rule_handle_var_instantiation(source, 16, self.mapping,
                                                                       self.ast_line_wise), result)

    def test_rule_replace_dot_with_arrow(self):
        source = r'chatter_pub.publish(msg);'
        result = r'chatter_pub->publish(msg);'
        self.assertEqual(self.cpp_porter.rule_replace_dot_with_arrow(source, 27, self.mapping, self.ast_line_wise),
                         result)

    def test_rule_replace_macros_with_node_arg(self):
        source = r'ROS_INFO("%s\n",msg.data.c_str());'
        result = r'RCLCPP_INFO(node->get_logger(),"%s\n",msg.data.c_str());'
        self.assertEqual(self.cpp_porter.rule_replace_macros(source, 25, self.mapping, self.ast_line_wise), result)
        

if __name__ == '__main__':
    unittest.main()