# ROS2 Migration Tools

This package contains tool to simplify migrating a package from ROS1 to ROS2. This package contains 3 tools: `docker_setup` for launching a docker container that has ROS2 and ROS1 installed, `check_deps` to check which dependencies have already been migrated to ROS2, and `porting_tools` which contain scripts to make some automatic changes to source code and build files and warn about manual changes.

Note: Any of the tools can be run with the `--help` option for the full list of options.

## Setup
### Docker
Install docker https://docs.docker.com/install/

### Folder Structure
 1. Create a new directory : `mkdir ros_migration_ws/src -p`
 2. Initialize a Catkin workspace: `cd ros_migration_ws/src ; catkin_init_workspace`
 2.	Fetch the package to be migrated into `ros_migration_ws/src/`
 3.	Clone this repo into `ros_migration_ws/ros2_migration_tools`

#### Install ros2_migration Python libraries
In `ros_migration_ws/ros2_migration_tools` run `pip3 install -e .`

## Checking Dependencies
From `ros_migration_ws/` run `python3 ../ros2_migration_tool.py check-deps <ros_package>`

If any dependencies haven't been migrated do those first

## Migrating a Package

### CMakeLists.txt and package.xml
1.	Run `python3 ros2_migration_tool.py catkin-to-ament --package-path <path/to/package/src>` (Note: by default this will overwrite your files!)
1.	Check the output to make sure the files were migrated successfully, note that comments may get messed up

Warning: Running the tool multiple times on the same file may result in duplicated lines

### Source Code
 1.	Run `python3 ros2_migration_tool.py port-cpp --src <path/to/ros1.cpp> --dst <path/to/ros2.cpp>`
 1.	Use `port-python` for python files

This will make some simple changes to the source code and produce warning for sections of code that may need to be changed. These warnings are not complete so even if no more warnings are generated the source code may still need changes

Source files cannot be completed migrated automatically and will also print warnings for lines that may need to be changed. These tools are incomplete. See the README in `porting_tools/` for how they can be extended.


### msg/srv files
The tool doesn't handle any changes to message or service declarations.

See [here](MISSINGLINK) for information on interfaces (messages and services) in ROS2. In particular note the section on field names: they must be be only lowercase and cannot end in underscores.

### launch files
In ROS2 launch files are python scripts not xml files.

Information about the ROS launch system can be found [here](https://github.com/ros2/ros2/wiki/Launch-system) and [here](https://github.com/bponsler/ros2-support/blob/master/tutorials/ros2-launch.md). Examples of launch files in ros2 can be found [here](https://github.com/ros2/launch)

### Docker container
Launch the docker container: `python3 ros2_migration_tool.py launch_docker --src-volume . --dst-dir /workspace`

## Specifying ROS installations

By default the docker setup uses ROS2 bouncy and ROS1 Melodic. ROS1 releases were tied
to an ubuntu release (for melodic 18.04) and ROS1 installations were only available for that release (you could also install from source). The ROS2 bouncy image used by default uses ubuntu 18.04 so melodic is used as the ROS1 release. If you want to use another version of ROS2 (using the `--tag` option) make sure to specify the appropriate ROS1 version (using the `--ros1-version` option). For example if you wanted to use ROS2 ardent (which comes with ubuntu 16.04) the appropriate ROS1 version would be kinetic and you would run the command:

`python3 ros2_migration_tool.py --tag:ardent-ros-base --ros1-version kinetic`

If you don't care about running the package in ROS1 then the ROS1 version shouldn't matter too much since it is only used for checking dependencies.



## Reference:

The migration tool is not able to do a complete migration itself. Please add or modify rules and warnings to improve it. Instructions for contributing can be found in porting_tools/README.md.

In general: follow the instructions for migration available at : https://github.com/ros2/ros2/wiki/Migration-Guide

Also look at these packages for example migrations:

•	For messages: [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs/pull/12/commits/437c5efdc5c6f013f270ca45faa985f3b5f7aaf2)

•	Some example nodes: [turtlebot3_nodes](https://github.com/ROBOTIS-GIT/turtlebot3/tree/ros2/turtlebot3_node/src)


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


