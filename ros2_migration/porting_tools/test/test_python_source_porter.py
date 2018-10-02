import unittest
from ros2_migration.porting_tools.python_source_porter import PythonSourcePorter 


test_script = """import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
"""

def get_line_nums(warnings):
    return [warning.line_number for warning in warnings]    

class TestPythonSourcePorter(unittest.TestCase):
    def test_warn_rospy(self):
        warnings = PythonSourcePorter.warn_rospy(test_script.split("\n"))
        lines = set([1,5,6,7,8,9,10,17])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines)

    def test_warn_publisher(self):
        warnings = PythonSourcePorter.warn_publisher(test_script.split("\n"))
        lines = set([5])
        warning_lines = set(get_line_nums(warnings))
        assert(lines==warning_lines)        

    def test_warn_subscriber(self):
        warnings = PythonSourcePorter.warn_subscriber(["rospy.Subscriber"])
        lines = set([1])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines)

    def test_warn_logging(self):
        warnings = PythonSourcePorter.warn_logging(test_script.split("\n"))
        lines = set([10])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines) 

    def test_warn_time(self):
        warnings = PythonSourcePorter.warn_time(["rospy.time"])
        lines = set([1])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines) 

    def test_warn_node_creation(self):
        warnings = PythonSourcePorter.warn_node_creation(test_script.split("\n"))
        lines = set([6])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines) 

    def test_warn_shutdown(self):
        warnings = PythonSourcePorter.warn_shutdown(test_script.split("\n"))
        lines = set([8])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines) 

    def test_warn_exceptions(self):
        warnings = PythonSourcePorter.warn_exceptions(test_script.split("\n"))
        lines = set([17])
        warning_lines = set(get_line_nums(warnings))
        self.assertEqual(lines, warning_lines) 

if __name__ == '__main__':
    unittest.main()
