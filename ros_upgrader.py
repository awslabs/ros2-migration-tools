from bonsai.cpp.clang_parser import CppAstParser
from porting_tools.cmake_lists_porter import CMakeListsPorter
from porting_tools.package_xml_porter import PackageXMLPorter
import xml.etree.ElementTree as etree
import os
import json
import shutil
import datetime
import fnmatch

class RosUpgrader:
    """
    Convert ROS1 package to ROS2 package
    """

    # path to the folder containing "compile_commands.json" file, will be generated using cmake flag
    # "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    COMPILE_DB_DIR = None

    # path to src folder of ROS1 package to convert to ROS2
    ROS1_SRC_PATH = None

    # path to all the include folders required by the package
    INCLUDES = []

    # output path where new src folder for the ROS2 package will be generated
    ROS2_OUTPUT_DIR = None

    # full path for the ROS2 package src
    ROS2_SRC_PATH = None

    @staticmethod
    def get_abs_path(rel_path):
        """
        Returns the absolute path for the given relative path
        :param rel_path:
        :return:
        """
        return os.path.abspath(rel_path)

    @staticmethod
    def init():
        """
        Initializes CppAstParser and Clang. Stores various config paths to be used later
        :return:
        """
        try:
            config = open("config.json")
        except IOError as e:
            print(e)

        config_obj = json.load(config)

        CppAstParser.set_library_path(RosUpgrader.get_abs_path(config_obj['LIBCLANG_PATH']))
        CppAstParser.set_database(RosUpgrader.get_abs_path(config_obj['COMPILE_DB_DIR']))
        RosUpgrader.ROS1_SRC_PATH = RosUpgrader.get_abs_path(config_obj['ROS1_SRC_PATH'])
        RosUpgrader.ROS2_OUTPUT_DIR = RosUpgrader.get_abs_path(config_obj['ROS2_OUTPUT_DIR'])
        RosUpgrader.INCLUDES = []
        RosUpgrader.COMPILE_DB_DIR = RosUpgrader.get_abs_path(config_obj['COMPILE_DB_DIR'])
        for fpath in config_obj['USER_INCLUDES']:
            RosUpgrader.INCLUDES.append(RosUpgrader.get_abs_path(fpath))

    @staticmethod
    def get_cpp_file_list():
        """
        Reads the "compile_commands.json" file from COMPILE_DB_DIR and get the list of .cpp files, which will be the
        entry points for parsing
        :return: list containing full path of all the .cpp files
        """
        file_list = []
        with open(os.path.join(RosUpgrader.COMPILE_DB_DIR, "compile_commands.json")) as f:
            entries = json.load(f)
            for entry in entries:
                if ".cpp" in entry["file"]:
                    file_list.append(entry["file"])

        return file_list

    @staticmethod
    def get_tokens_as_json():
        """
        Creates an instance of CppAstParser and gets all the tokens for various .cpp files as a python dict
        :return: dict with key as path to .cpp and values as the tokens dict
        """
        cp = CppAstParser(RosUpgrader.ROS1_SRC_PATH, RosUpgrader.INCLUDES)

        file_list = RosUpgrader.get_cpp_file_list()

        all_tokens = {}
        for file_path in file_list:
            all_tokens[file_path] = cp.get_ast_obj(file_path)

        return all_tokens

    @staticmethod
    def copy_directory(src, dest):
        """
        Copies the contents of src to dest
        :param src: path to folder to copy
        :param dest: path to folder where to copy
        :return: None
        """
        try:
            shutil.copytree(src, dest)
        # Directories are the same
        except shutil.Error as e:
            print('Directory not copied. Error: %s' % e)
        # Any error saying that the directory doesn't exist
        except OSError as e:
            print('Directory not copied. Error: %s' % e)

    @staticmethod
    def init_ros2_folder():
        """
        Creates a new folder in ROS2_OUTPUT_DIR, with name src_currDate_currTime, and then copies the contents of ROS1
        package from ROS1_SRC_PATH to the created directory
        :return:
        """
        curr_folder_id = '{date:%Y-%m-%d_%H_%M_%S}'.format( date=datetime.datetime.now())
        RosUpgrader.ROS2_SRC_PATH = os.path.join(RosUpgrader.ROS2_OUTPUT_DIR, "src_" + curr_folder_id)
        RosUpgrader.copy_directory(RosUpgrader.ROS1_SRC_PATH, RosUpgrader.ROS2_SRC_PATH)

    @staticmethod
    def get_all_file_paths(file_name):
        """
        Recursively find all the file with name 'file_name' in folder ROS2_SRC_PATH
        :param file_name: name of the file to search recursively
        :return: list containing file paths
        """
        matches = []
        for root, dirnames, filenames in os.walk(RosUpgrader.ROS2_SRC_PATH):
            for file in fnmatch.filter(filenames, file_name):
                matches.append(os.path.join(root, file))
        return matches

    @staticmethod
    def convert_all_cmake():
        """
        Converts all the cmake files from ROS1 to ROS2
        :return: None
        """
        cmake_files_list = RosUpgrader.get_all_file_paths("CMakeLists.txt")

        for file_path in cmake_files_list:
            with open(file_path, 'r') as src_file:
                ported_content = CMakeListsPorter.port(content=src_file.read())
                with open(file_path, 'w') as new_cmake:
                    new_cmake.write(ported_content)

    @staticmethod
    def convert_all_package_xml():
        """
        Converts all package.xml files from ROS1 to ROS2
        :return: None
        """
        package_files_list = RosUpgrader.get_all_file_paths("package.xml")

        for file_path in package_files_list:
            tree = etree.parse(file_path)
            PackageXMLPorter.port(tree=tree)
            tree.write(file_path, encoding="utf-8", xml_declaration=True)

    def __init__(self):
        pass



