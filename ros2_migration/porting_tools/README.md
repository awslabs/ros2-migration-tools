

This is a tool for automatically converting a ROS1 package to a ROS2 package.

This tool is incomplete! Especially the source code changes and warnings. If you come across 
something that is missing please see the guide below to add it!



Currently the script makes the following changes:


## Package.xml:

-set format to 3

-changes client library for python and c++ (i.e. rospy->rclpy, roscpp->rclcpp)

-add build_depend rmw_implementation

-replace message generation build dependency with build tool depend rosidl_default_generators and exec depend rosidl_default_runtime

-replace message generation run_dependancy with build_tool depend rosidl_default_generators and exec depend rosidl_default_runtime

-replace all run_depend with exec_depend

-replace build_tool depend catkin with buildtool_depend ament_cmake

-Adds <export><build_type>ament_cmake</build_type></export> to bottom of file 

-replace depend with exec_depend and build_depend


## CMakeLists.txt:
-Set cmake minimum required version to 3.5

-Set to c++14 std

-Split up find_package

-Remove unused commands

-Replaces renamed commands

-Add rmw_implementation as required package

-Call ament_package at bottom instead of catkin_package

-Include directories separately

-Target link libraries separately

-Install directory relative to installation root

-Changes message generation and service generation to interface generation

## C++ Source Code:
-see constants.CPPSourceChanges.CPP_SOURCE_REPLACEMENTS for the regex replacements

-warnings

## Python Source Code:
- no changes

- warnings

## Contributing:

There are four classes for porting: CPPSourcePorter, PythonSourcePorter, CMakeListsPorter, PackageXMLPorter (feel free to add another for different file types)
They all have the same structure:

The main method is port(<input>, extra_rules=[], extra_warnings=[]).

The type of <input> varies between the classes (see the class specific instructions below).

The port method applies "rules" and "warnings" to <input> and then returns the modified input.

Rules make changes to <input>. If <input> is an immutable object (e.g. a string) the rule should return the new object otherwise it doesn't return anything.

A rule is any static method of the porting class whose name starts with "rule". Rules are expected be applied independently of any other rule and order of application is not guaranteed.

To add a rule, just add a method like so:
```python
@staticmethod
def rule_<name>(<input>):
	#do stuff
```

Warnings do not make changes to <input> and can either print a message or return a list of MigrationWarning objects (see utils.py) depending on the porting class.

Warnings are applied after rules. To add a warning just add a method like so:

```python
@staticmethod
def warn_<name>(<input>):
	#do stuff
	return <list of MigrationWarnings>
```

##CPPSourcePorter

- The <input> type is a single string for the cpp source file, i.e. the output of file.read()

- Rules should return a modified string.

- Warnings will get a list of lines (i.e. source.split("\n"))

- Warnings return a list of MigrationWarnings

- You can use the find_warnings method in utils.py to more easily create MigrationWarnings

## PythonSourcePorter

- Same as CPPSource Porter

## CMakeListsSourcePorter
- This porter relies on the parse_cmake package available at: https://github.com/wjwwood/parse_cmake

This library represents a CMakeLists file as a list of "Command" (created with the Command(args) function), BlankLine, and Comment objects.

- <input> is a string with the contents of CMakeLists.txt (i.e. the output for file.read()) 

- The input is then parsed into a list of Command, Blankline and Comment objects which is then passed to the rules and warnings

- A Command object takes a name and then a list of arguments

- For example the command: find_package(ament_cmake REQUIRED) is represented as Command('find_package', [Arg('ament_cmake'), Arg('REQUIRED')])

- you can use the function commands_with(name, from_cmake) to filter for commands with a certain name e.g. all "find_package" commands

- Since there isn't necessarily a 1:1 mapping between lines and indices in the list, warnings just print out a message


## PackageXMLPorter

- This porter relies on the element tree xml parser: https://docs.python.org/3.6/library/xml.etree.elementtree.html

- <input> is an xml tree (i.e. the result of etree.parse(<file>))
	
- Rules take as input tree.getRoot() and modify the tree from there

- Any warnings should print out a message








