
This tool will pull a docker image, set it up, and launch it in your terminal.

To install necessary packages run:
$ pip install -e .
in the docker_setup folder

For usage instructions run:
$ python3 docker_setup.py -h

Specifying ROS installations:

By default the docker setup uses ROS2 bouncy (the latest release) and ROS1 Melodic. ROS1 releases were tied
to an ubuntu release (for melodic 18.04) and ROS1 installations were only available for that release (you
could also install from source). The ROS2 bouncy image used by default uses ubuntu 18.04 so melodic is
used as the ROS1 release. If you want to use another version of ROS2 (using the --tag option) make sure to 
specify the appropriate ROS1 version (using the --ros1-version option). For example if you wanted to use ROS2
ardent (which comes with ubuntu 16.04) the appropriate ROS1 version would be kinetic and you would run the command:
python3 ros2_migration_tool.py --tag:ardent-ros-base --ros1-version kinetic

If you don't care about running the package in ROS1 then the ROS1 version shouldn't matter too much since it is
only used for checking dependencies.

