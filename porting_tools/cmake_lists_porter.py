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

"""
A class for porting CMakeLists from ROS1 to ROS2
"""
import os.path

try:
    import parse_cmake.parsing as cmkp
except ImportError:
    import traceback
    traceback.print_exc()
    print("ERROR: You must install the parse_cmake python module!")
    print("")
    print("pip3 install parse_cmake")
    exit(1)

from .constants import CatkinToAmentMigration, ROS1ROS2Packages
from .utils import get_functions_with


def extract_args(item):
    """ Returns the strings from an object created by the cmake parser """
    args = [arg.contents for arg in item.body]
    return args


def commands_with(name, from_cmake):
    """ Returns a list of all cmkp._Command objects from a cmakeLists with the given name. """
    cmd_list = []
    #yield from ((index, command) for (index, command) in enumerate(from_cmake) if (isinstance(command, cmkp._Command) and command.name == name))
    for (index, command) in enumerate(from_cmake):
        if isinstance(command, cmkp._Command) and command.name == name:
            yield (index, command)


class CMakeListsPorter():
    """ A class of static methods for porting CMakeLists from ROS1 to ROS2 """

    @staticmethod
    def port(content, extra_rules=[], extra_warnings=[]):
        """
        Arguments:
            content - A string with contents of CMakeLists.txt
            extra_rules - a list of functions (rules) to apply to the content
            extra_warnings - a list of functions to check for warnings in the content

        Returns:
            A new string with the updated CMakeLists
        """


        cmake = cmkp.parse(content)

        #Pulls out all methods in this class with name starting with "rule"
        rules = get_functions_with(criteria=lambda name: name.startswith("rule"),
                                   from_class=CMakeListsPorter)

        for rule in rules + extra_rules:
            rule(cmake)

        #Pulls out all methods in this class with name starting with "warn"
        warnings = get_functions_with(criteria=lambda name: name.startswith("warn"),
                                      from_class=CMakeListsPorter)

        for warning in warnings + extra_warnings:
            warning(cmake)

        # This command must be at the bottom of the package
        cmake.append(cmkp.BlankLine())
        ament_package_cmd = cmkp.Command("ament_package", [])
        cmake.append(ament_package_cmd)

        cmake_contents = '\n'.join(cmkp.compose_lines(cmake, cmkp.FormattingOptions())) + '\n'
        #Post-process text
        #replace variable names:
        for var, replacement in CatkinToAmentMigration.CMAKE_LISTS_RENAMED_VARIABLES.items():
            cmake_contents = cmake_contents.replace(var, replacement)

        return cmake_contents



    #########################
    #          HELPERS      #
    #########################
    @staticmethod
    def set_cpp_std_definition(cmake):
        """
        Check if c++ standard is set in definitions, update to c++14 if so
        returns true if the standard was succesfully set
        This is a helper function for rule_set_cpp_std, not a rule itself
        """
        cpp_std_set = False
        for _, item in commands_with(name="add_definitions", from_cmake=cmake):
            for index, arg in enumerate(item.body):
                if "-std=c++" in arg.contents:
                    item.body[index] = cmkp.Arg("-std=c++14")
                    cpp_std_set = True
        return cpp_std_set


    @staticmethod
    def set_cpp_std_flag(cmake):
        """
        Check if c++ standard is set with a cmake flag, update to c++14 if so
        returns true if the standard was successfully set
        This is a helper function for rule_set_cpp_std, not a rule itself
        """
        cpp_std_set = False
        for _, item in commands_with(name="set", from_cmake=cmake):
            for index, arg in enumerate(item.body):
                if "-std=c++" in arg.contents:
                    item.body[index] = cmkp.Arg("-std=c++14")
                    cpp_std_set = True
                #Check that CMAKE_CXX_STANDARD isn't the last argument
                elif "CMAKE_CXX_STANDARD" in arg.contents and len(item.body) > index+1:
                    item.body[index+1] = cmkp.Arg("14")
                    cpp_std_set = True
        return cpp_std_set

    @staticmethod
    def get_catkin_dependencies(cmake):
        """
        Retreives the dependences in the find_package command in catkin cmake
        This is a helper function for rule_update_dependencies, not a rule itself
        """
        catkin_depends = []

        for _, find_package_item in commands_with(name="find_package", from_cmake=cmake):
            args = extract_args(item=find_package_item)
            if "COMPONENTS" in args:
                depends = args[args.index("COMPONENTS")+1:]
            else:
                depends = args

            # Remove packages that no longer exist in ROS 2
            depends = list(filter(
                lambda p: p not in ROS1ROS2Packages.UNKNOWN_ROS_PACKAGES,
                depends))

            # Handle packages that have been renamed in ROS 2
            depends = list(map(
                lambda p: ROS1ROS2Packages.RENAMED_ROS_PACKAGES.get(p, p),
                depends))

            #Add additional packages needed for message generation
            if "message_generation" in args:
                depends.extend([
                    "rosidl_default_generators",
                ])
            catkin_depends.extend(depends)

        # Remove duplicate entries
        catkin_depends = list(set(catkin_depends))

        catkin_depends.insert(0, "ament_cmake")

        # Remove packages that don't need to be found by CMake
        for pkg in ROS1ROS2Packages.NO_CMAKE_FIND_PACKAGES:
            if pkg in catkin_depends:
                catkin_depends.remove(pkg)
        return catkin_depends

    @staticmethod
    def add_ament_dependencies(cmake, dependencies, insert_index):
        """
        Adds dependencies at provided index
        returns the next the next index
        This is a helper function for rule_update_dependencies not a rule itself
        Arguments:
            cmake - the parsed CMakeList
            dependencies - the dependencies from the CMakeLists (output from get_catkin_dependencies)
            insert_index - the index in the cmake to start inserting dependency commands

        Returns:
            The next index after the dependencie
        """

        # Define variables for all include dirs and libraries
        include_args = [cmkp.Arg("INCLUDE_DIRS")]
        lib_args = [cmkp.Arg("LIBS")]
        lib_dir_args = [cmkp.Arg("LIBRARY_DIRS")]
        for pkg in dependencies:
            include_args.append(cmkp.Arg("${%s_INCLUDE_DIRS}" % pkg))
            lib_args.append(cmkp.Arg("${%s_LIBRARIES}" % pkg))
            lib_dir_args.append(cmkp.Arg("${%s_LIBRARIES}" % pkg))

        # If an include directory exists for this package, add it to
        # the include dirs
        if os.path.exists("include"):
            include_args.insert(1, cmkp.Arg("include"))

        for pkg in dependencies:
            find_command = cmkp.Command("find_package", [cmkp.Arg(pkg), cmkp.Arg("REQUIRED")])
            cmake.insert(insert_index, find_command)
            insert_index += 1

        # Add command to set include dirs
        set_include_dirs = cmkp.Command("set", include_args)
        cmake.insert(insert_index, set_include_dirs)
        insert_index += 1

        ## Add the include_directories command
        include_dirs = cmkp.Command(
            "include_directories", [cmkp.Arg("${INCLUDE_DIRS}")])
        cmake.insert(insert_index, include_dirs)
        insert_index += 1

        # Add command to set lib dirs
        set_lib_dirs = cmkp.Command("set", lib_dir_args)
        cmake.insert(insert_index, set_lib_dirs)
        insert_index += 1

        ## Add the link_libraries command
        #Get executable names:
        for _, item in commands_with(name="add_executable", from_cmake=cmake):
            executable = extract_args(item=item)[0]
            link_dirs = cmkp.Command(
                "target_link_libraries", [cmkp.Arg(executable), cmkp.Arg("${LIBRARY_DIRS}")])
            cmake.insert(insert_index, link_dirs)
            insert_index += 1

        # Add command to set libs
        set_libs = cmkp.Command("set", lib_args)
        cmake.insert(insert_index, set_libs)
        insert_index += 1

        return insert_index


    @staticmethod
    def update_messages_services(cmake, msg_dependencies, insert_index):
        """
        Updates commands for defining messages and services in cmake

        Arguments:
            cmake - parsed CMakeList
            msg_dependencies - list of the names of msg and service dependencies to be added
            insert_index - the index to insert the rosidl_generate_interfaces command
        Returns:
            None
        """
        msgs_services = []
        for _, item in commands_with(name="add_message_files", from_cmake=cmake):
            args = extract_args(item=item)
            if len(args) > 1:
                msgs_services.extend(list(map(lambda s: "msg/%s" % s, args[1:])))
        for _, item in commands_with(name="add_service_files", from_cmake=cmake):
            args = extract_args(item=item)
            if len(args) > 1:
                msgs_services.extend(list(map(lambda s: "srv/%s" % s, args[1:])))

        if msgs_services:
            cmd_args = [cmkp.Arg("${PROJECT_NAME}")]
            for file_name in msgs_services:
                cmd_args.append(cmkp.Arg('"%s"' % file_name))

            # Add dependencies on message packages
            cmd_args.extend([
                cmkp.Arg("DEPENDENCIES"),
                cmkp.Arg("builtin_interfaces"),
            ])

            for pkg in msg_dependencies:
                cmd_args.append(cmkp.Arg(pkg))

            interface_command = cmkp.Command("rosidl_generate_interfaces", cmd_args)

            cmake.insert(insert_index, interface_command)

        CMakeListsPorter.delete_commands_with(names=["add_message_files", "add_service_files"], from_cmake=cmake)


    @staticmethod
    def delete_commands_with(names, from_cmake):
        """ Remove all commands with a name in names from from_cmake"""
        remove_criteria = lambda pair: isinstance(pair[1], cmkp._Command) and pair[1].name in names
        remove_indices = [pair[0] for pair in filter(remove_criteria, enumerate(from_cmake))]
        # Remove all indices in reverse sorted order to prevent the
        # indices from changing values
        for index in sorted(remove_indices, reverse=True):
            del from_cmake[index]


    #########################
    #        RULES          #
    #########################

    @staticmethod
    def rule_set_cmake_version(cmake):
        """ Set the cmake version to 3.5 """
        try:
            _, cmake_min_statements = next(commands_with(name="cmake_minimum_required", from_cmake=cmake))
            if len(cmake_min_statements.body) == 2 and cmake_min_statements.body[0].contents == "VERSION":
                cmake_min_statements.body[1] = cmkp.Arg("3.5")
        except StopIteration: #No cmake minimum was set
            cmake.insert(0, cmkp.Command("cmake_minimum_required", [cmkp.Arg("VERSION"), cmkp.Arg("3.5")]))

    @staticmethod
    def rule_set_cpp_std(cmake):
        """ Set std=c++14 """
        if not (CMakeListsPorter.set_cpp_std_definition(cmake) or CMakeListsPorter.set_cpp_std_flag(cmake)):

            #Create c++ standard
            command = [cmkp.BlankLine(),
                       cmkp.Comment("# Add support for C++14"),
                       cmkp.Command("if", [cmkp.Arg("NOT"), cmkp.Arg("CMAKE_CXX_STANDARD")]),
                       cmkp.Command("set", [cmkp.Arg("CMAKE_CXX_STANDARD"), cmkp.Arg("14")]),
                       cmkp.Command("endif", []),
                       cmkp.BlankLine()
                       ]

            try:
                insert_index, _ = next(commands_with(name="project", from_cmake=cmake))
            except StopIteration:
                raise ValueError("Unable to find project declaration in CMakeLists")

            for increment, item in enumerate(command):
                cmake.insert(insert_index+1+increment, item)

    @staticmethod
    def rule_update_dependencies(cmake):
        """ Adds commands required to find, include, and link dependencies"""
        catkin_depends = CMakeListsPorter.get_catkin_dependencies(cmake)
        #catkin_depends.append("rmw_implementation")

        try:
            find_packages = next(commands_with(name="find_package", from_cmake=cmake))
        except StopIteration:
            find_packages = None

        #Delete old dependencies
        deleteable_commands = ["find_package",
                               "include_directories",
                               "target_link_libraries"]

        CMakeListsPorter.delete_commands_with(names=deleteable_commands, from_cmake=cmake)

        #The add_executable command must come before the target_link_libraries command
        #It's okay if it comes before find_packages and include_directories
        #so all of these commands are just placed after add_executable
        add_executables = list(commands_with(name="add_executable", from_cmake=cmake))

        if add_executables:
            insert_index = add_executables[-1][0]
        elif find_packages: #There were no add_executables so just insert where the old find_package was
            insert_index = find_packages[0]
        else: #no find_packages either so use the project declaration which all valid CMakeLists should have
            insert_index, _ = next(commands_with(name="project", from_cmake=cmake))

        next_index = CMakeListsPorter.add_ament_dependencies(cmake=cmake, dependencies=catkin_depends, insert_index=insert_index+1)

        msg_dependencies = list(filter(lambda pkg: pkg.endswith("_msgs") or pkg.endswith("srvs"), catkin_depends))

        CMakeListsPorter.update_messages_services(cmake=cmake, msg_dependencies=msg_dependencies, insert_index=next_index)

        #Export dependencies
        cmake.append(cmkp.BlankLine())
        for pkg in catkin_depends:
            cmake.append(cmkp.Command("ament_export_dependencies", [cmkp.Arg(pkg)]))

    @staticmethod
    def rule_remove_unused_commands(cmake):
        """
        Removes commands that aren't necessary in ament.
        The commands to remove are listed in constants.py
        """
        CMakeListsPorter.delete_commands_with(names=CatkinToAmentMigration.CMAKE_LISTS_DELETED_COMMANDS, from_cmake=cmake)


    @staticmethod
    def rule_replace_commands(cmake):
        """
        Replaces commands that have been renamed between catkin and ament
        List of commands are found in constants.py
        """
        for item in filter(lambda item: isinstance(item, cmkp._Command), cmake):
            if item.name in CatkinToAmentMigration.CMAKE_LISTS_REPLACED_COMMANDS:
                item.name = CatkinToAmentMigration.CMAKE_LISTS_REPLACED_COMMANDS[item.name]


    @staticmethod
    def rule_add_export_libs(cmake):
        """ Adds export include libs to cmake """
        cmake.append(cmkp.Command("ament_export_include_directories", [cmkp.Arg("${INCLUDE_DIRS}")]))

        package_libs = []
        for _, item in commands_with(name="add_library", from_cmake=cmake):
            if item.body:
                package_libs.append(item.body[0].contents)

        if package_libs:
            export_libs = cmkp.Command("ament_export_libraries", [cmkp.Arg(lib) for lib in package_libs])
            # Include all dependency libraries
            export_libs.body.append(cmkp.Arg("${LIBS}"))
            cmake.append(export_libs)


    @staticmethod
    def rule_replace_target_link_libs(cmake):
        """ Replaces catkin_Libraries with LIBS """
        for _, item in commands_with(name="target_link_libraries", from_cmake=cmake):
            args = extract_args(item=item)
            if "${catkin_LIBRARIES}" in args:
                catkin_index = args.index("${catkin_LIBRARIES}")
                item.body[catkin_index] = cmkp.Arg("${LIBS}")


    #########################
    #      WARNINGS         #
    #########################

    @staticmethod
    def warn_pure_python(cmake):
        """ Prints a warning if a package is potentially pure python """
        msgs_srvs = list(commands_with(name="add_message_files", from_cmake=cmake))
        msgs_srvs.extend(commands_with(name="add_service_files", from_cmake=cmake))
        msgs_srvs.extend(commands_with(name="rosidl_generate_interfaces", from_cmake=cmake))

        dependencies = CMakeListsPorter.get_catkin_dependencies(cmake)
        uses_python = "rclpy" in dependencies or "rospy" in dependencies
        uses_cpp = "rclcpp" in dependencies or "roscpp" in dependencies

        if not msgs_srvs and uses_python and not uses_cpp:
            print("WARNING: Package may be a pure python package because \
                it doesn't generate any messages or services and only uses python.\
                If this is the case a CMakeLists shouldn't be used.\
                see https://github.com/ros2/ros2/wiki/Migration-Guide")






if __name__ == '__main__':
    with open("CMakeLists.txt", 'r') as file:
        content = file.read()
    with open("ros2_CMakeLists.txt", 'w') as output:
        output.write(CMakeListsPorter.port(content=content))
