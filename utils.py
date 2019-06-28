from Constants import Constants
import os
import shutil
import datetime
import json

class Utilities:
    """
    Provides some utility functions
    """
    # path to src folder of ROS1 package to convert to ROS2
    ROS1_SRC_PATH = None

    # output path where new src folder for the ROS2 package will be generated
    ROS2_OUTPUT_DIR = None

    # full path for the ROS2 package src
    ROS2_SRC_PATH = None

    @staticmethod
    def init():
        """
        Initializes CppAstParser and Clang. Stores various config paths to be used later
        :return:
        """
        try:
            config = open(Constants.CONFIG_FILE_NAME)
        except IOError as e:
            print(e)

        config_obj = json.load(config)

        Utilities.ROS1_SRC_PATH = Utilities.get_abs_path(config_obj[Constants.ROS1_SRC_PATH])
        Utilities.ROS2_OUTPUT_DIR = Utilities.get_abs_path(config_obj[Constants.ROS2_OUTPUT_DIR])

    @staticmethod
    def get_abs_path(rel_path):
        """
        Returns the absolute path for the given relative path
        :param rel_path:
        :return:
        """
        return os.path.abspath(rel_path)

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
    def get_uniquie_id():
        """
        Generates a unique id
        :return: string
        """
        return '{date:%Y-%m-%d_%H_%M_%S}'.format(date=datetime.datetime.now())

    @staticmethod
    def get_parent_dir(directory):
        """
        Gets the parent directory for give path
        :param directory:
        :return: str
        """
        return os.path.abspath(os.path.join(directory, os.pardir))

    @staticmethod
    def init_ros2_folder():
        """
        Creates a new folder in ROS2_OUTPUT_DIR, with name src_currDate_currTime, and then copies the contents of ROS1
        package from ROS1_SRC_PATH to the created directory
        :return:
        """
        Utilities.ROS2_SRC_PATH = os.path.join(Utilities.ROS2_OUTPUT_DIR, Utilities.get_uniquie_id(), Constants.ROS2_SRC_FOLDER)
        Utilities.copy_directory(Utilities.ROS1_SRC_PATH, Utilities.ROS2_SRC_PATH)

if __name__ == "__main__":
    Utilities.init()
    Utilities.init_ros2_folder()
    print(Utilities.ROS2_SRC_PATH)