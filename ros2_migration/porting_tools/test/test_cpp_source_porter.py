import unittest
from ros2_migration.porting_tools.cpp_source_porter import CPPSourcePorter


test_source = """#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  std_msgs::String msg;
  while (ros::ok())
  {
    std::stringstream ss;
    ss << "hello world " << count++;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
"""



def get_line_nums(warnings):
    return [warning.line_number for warning in warnings]   

class TestCPPSourcePorter(unittest.TestCase):

    def test_rule_replace_simple_include_msg(self):
        source = '#include std_msgs/String.h'
        result = '#include std_msgs/msg/string.hpp'
        self.assertEqual(CPPSourcePorter.rule_replace_simple(source), result)

    def test_rule_replace_simple_msg_namespace(self):
        source = 'std_msgs::String'
        result = 'std_msgs::msg::String'
        self.assertEqual(CPPSourcePorter.rule_replace_simple(source), result)

    def test_warn_ros_logging(self):
        warnings = CPPSourcePorter.warn_ros_logging(test_source.split("\n"))
        lines = set([17])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines)   

    def test_warn_ros_time(self):
        warnings = CPPSourcePorter.warn_ros_time(["ros::Time begin = ros::Time::now();"])
        lines = set([1])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines)  

    def test_warn_msg_import(self):
        warnings = CPPSourcePorter.warn_msg_import(test_source.split("\n"))
        lines = set([3])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines)  

    def test_warn_publish(self):
        warnings = CPPSourcePorter.warn_publish(test_source.split("\n"))
        lines = set([18])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines) 

    def test_warn_subscribe(self):
        warnings = CPPSourcePorter.warn_subscribe(['ros::Subscriber sub = nh.subscribe("my_topic", 1, callback, ros::TransportHints().unreliable());'])
        lines = set([1])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines) 

    def test_warn_node_spin(self):
        warnings = CPPSourcePorter.warn_node_spin(test_source.split("\n"))
        lines = set([19])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines) 

    def test_warn_node_decl(self):
        warnings = CPPSourcePorter.warn_node_decl(test_source.split("\n"))
        lines = set([7])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines) 

    def test_warn_publish_creation(self):
        warnings = CPPSourcePorter.warn_publish_creation(test_source.split("\n"))
        lines = set([8])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines) 

    def test_warn_msg_data(self):
        warnings = CPPSourcePorter.warn_msg_data(test_source.split("\n"))
        lines = set([16,17])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines) 

    def test_warn_node_param(self):
        warnings = CPPSourcePorter.warn_node_param(['nh.getParam("relative_name", relative_name)',
                                                    'nh.param<std::string>("default_param", default_param, "default_value");',
                                                    'nh.setParam("/global_param", 5);',
                                                    'nh.hasParam("my_param")',
                                                    'nh.deleteParam("my_param");'])
        lines = set([1, 2, 3, 4, 5])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines) 



if __name__ == '__main__':
    unittest.main()
