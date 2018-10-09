# ROS2 Migration Tools

This package contains tool to simplify migrating a package from ROS1 to ROS2.

How to build:

In the root directory run pip3 install -e .

How to run:

This package contains 3 tools: docker_setup for launching a docker container that has ROS2 and ROS1 installed, check_deps to check which dependencies have already been migrated to ROS2, and porting_tools which contain scripts to make some automatic changes to source code and build files and warn about manual changes.

For options run:

Python3 ros2_migration_tool.py --help

## Docker Setup

For help run: python3 ros2_migration_tool.py launch_docker --help

### Specifying ROS installations:

By default the docker setup uses ROS2 bouncy (the latest release) and ROS1 Melodic. ROS1 releases were tied
to an ubuntu release (for melodic 18.04) and ROS1 installations were only available for that release (you could also install from source). The ROS2 bouncy image used by default uses ubuntu 18.04 so melodic is used as the ROS1 release. If you want to use another version of ROS2 (using the --tag option) make sure to specify the appropriate ROS1 version (using the --ros1-version option). For example if you wanted to use ROS2 ardent (which comes with ubuntu 16.04) the appropriate ROS1 version would be kinetic and you would run the command:

python3 ros2_migration_tool.py --tag:ardent-ros-base --ros1-version kinetic

If you don't care about running the package in ROS1 then the ROS1 version shouldn't matter too much since it is only used for checking dependencies.

Checking Dependencies

For help run: python3 ros2_migration_tool.py check_deps --help

In the root directory of the ros package run python3 ros2_migration_tool.py check_deps <Package Name>
 
This will print out a list of dependencies and their migration status if known.

## Migrating a Package

For migrating package.xml and CMakeLists.txt files from catkin to ament run:

Python3 ros2_migration_tool.py catkin_to_ament --help 

Source files cannot be completed migrated automatically and will also print warnings for lines that may need to be changed. These tools are incomplete. See the README in porting_tools/ for how they can be extended.

For migrating C++ source files run:

Python3 ros2_migration_tool.py port_cpp --help

For migrating python source files run:

Python3 ros2_migration_tool.py port_python --help

## Package Migration Steps:

1.	Get the migration tooling 
2.	Install docker https://docs.docker.com/install/
3.	Setup a workspace:
    1. Create a new directory : mkdir ros_migration_ws
    2.	Into this directory fetch the package to be migrated
    3.	Also move the migration tools into this directory
    4.	You should now have a workspace with ros_migration_ws/ros2_migration_tools, ros_migration_ws/<ros_package>
4.	Launch the docker container: python3 ros2_migration_tool.py launch_docker --src-volume . --dst-dir /workspace
    1.	By default this uses bouncy (the latest ROS2 release) and melodic (the ROS1 release for ubuntu 18.04). Use python3 ros2_migration_tool.py launch_docker --help  if you need to specify versions of ROS2 or ROS1 (also see docker_setup/README)
5.	Check which dependencies have been migrated
    1.	From the ros package cd /workspace/<ros_package>
    1.	 python3 ../ros2_migration_tool.py check_deps <ros_package> (Note that this must be run inside the directory containing the package)
    1.	If any dependencies haven't been migrated do those first
6.	Migrate the CMakeLists.txt and package.xml files:
    1.	Run python3 ros2_migration_tool.py catkin_to_ament --package-path <path/to/package/src> (Note: by default this will overwrite your files!)
    1.	Run  python3 ros2_migration_tool.py catkin_to_ament --help for options to not overwrite the current files
    1.	Check the output to make sure the files were migrated successfully, note that comments may get messed up
    1.	Warning: Running the tool multiple times on the same file may result in duplicated lines
    1.	See the CMakeLists and package.xml sections below for more information on what the tool does.
7.	Migrate the source code 
    1.	Run python3 ros2_migration_tool.py port_cpp --src <path/to/ros1.cpp> --dst <path/to/ros2.cpp>
    1.	Use port_python for python files
    1.	This will make some simple changes to the source code and produce warning for sections of code that may need to be changed. These warnings are not complete so even if no more warnings are generated the source code may still need changes
    1.	See the C++ source code changes and python source code change sections below for more information on what the tool does and what needs to be done by hand.
8.	Migrate any message or service files
    1.	The tool doesn't handle any changes to message or service declarations
    1.	See here for information on interfaces (messages and services) in ROS2
    1.	In particular note the section on field names: they must be be only lowercase and cannot end in underscores
9.	Migrate any launch files
    1.	In ROS2 launch files are python scripts not xml files
    1.	Information about the ROS launch system can be found here and here
    1.	Examples of launch files in ros2 can be found here


## Reference:

The migration tool is not able to do a complete migration itself. Please add or modify rules and warnings to improve it. Instructions for contributing can be found in porting_tools/README.md.

In general: follow the instructions for migration available at : https://github.com/ros2/ros2/wiki/Migration-Guide

Also look at these packages for example migrations:

•	For messages: turtlebot3_msgs

•	Some example nodes: turtlebot3_nodes


### CMakeLists.txt Changes

These are the changes made by the migration tool to the CMakeLists
•	Set the Cmake version minimum to 3.5

•	Set the C++ standard for compiling to C++14

•	Removes packages that are no longer used in ROS2

•	Splits up the find_packages command

•	Adds rosidl_default_generators and rosidl_default_runtime instead of message_generation

•	Adds ament_cmake as a required package

•	Replaces add_message_files and add_service_files with rosidl_generate_interfaces

•	Replaces renamed commands (list can be found in porting_tools/constants.py)

•	Warns about using CMakeLists for a pure python package
 package.xml Changes

These are the changes made by the migration tool to the package.xml.
•	Sets format to 3

•	Sets build_tool_dependencies on rmw_implementation and cmake

•	Adds build_depend and exec_depend on rmw_implementation

•	Converts the client library (e.g. roscpp→rclcpp)

•	Adds a tag to export ament_cmake as the build type

•	changes all run_depends to exec_depends

•	Turns all “depends” into build_depend and exec_depend

•	Replaces message generation with build and exec depends on rosidl_default_generators and rosidl_Default_runtime

Changes that need to be done by hand:

•	Any spacing/formatting issues

### C++ Source Code Changes

These changes are made by the migration tool
•	Replaces many simple patterns (see porting_tools/constants.py CPP_SOURCE_REPLACEMENTS for a complete list), including:

o	simple message creation

o	simple node instantiation

•	Warns on a number of patterns. The warnings are just using simple pattern matching  to look for pieces of code that may need to be changed and so some may be erroneous.

Changes that may need to be done by hand:

•	Anything highlighted by the warnings

•	Node initialization

•	Creating messages and setting message data

•	Ensuring that messages are pass as shared pointers

•	Publishing/Subscribing to topics

•	Any changes to header files

### Python Source Code Changes

The major changes are moving from rospy to rclpy and that ROS2 only supports python3

The rclpy client library is less mature than rclcpp and at the time of writing is lacking some features (like a complete time package or the is_shutdown() function). There is also no centralized documentation on the changes between rospy and rclpy. The source code for rclpy can be found here and rospy documentation can be found here. 

Python2 → Python3: Use the 2to3 tool on any python2 source files in the package. Note that there may be some weird results if you use the tool on a python3 file.

Changes that may need to be done by hand:

•	Changing logging

•	Node initialization

•	Exceptions

•	rclpy api differences


