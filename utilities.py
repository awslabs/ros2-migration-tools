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
            raise e

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
            raise e
        # Any error saying that the directory doesn't exist
        except OSError as e:
            raise e

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
    def read_from_file(full_file_path):
        """
        Read file and return file handle
        :param full_file_path:
        :return: file contents as string
        """
        try:
            with open(full_file_path, 'r') as f:
                contents = f.read()
                f.close()
                return contents
        except IOError as e:
            raise e

    @staticmethod
    def write_to_file(full_file_path, contents):
        """
        Gets write file handle for full_file_path
        :param full_file_path:
        :return: None
        """
        try:
            with open(full_file_path, 'w') as f:
                f.write(contents)
                f.close()
        except IOError as e:
            raise e

    @staticmethod
    def read_json_file(full_file_path):
        """
        Read json file with path: full_file_path
        :param full_file_path:
        :return: json object
        """
        try:
            with open(full_file_path, 'r') as f:
                json_data = json.load(f)
                f.close()
                return json_data
        except IOError as e:
            raise e

    @staticmethod
    def write_as_json(full_file_path, json_data, indent=4):
        """
        Write json_data to file: full_file_path
        :param full_file_path:
        :param json_data:
        :return: None
        """
        try:
            with open(full_file_path, 'w') as f:
                json.dump(json_data, f, indent=indent, sort_keys=True)
        except IOError as e:
            raise e

    @staticmethod
    def simple_replace(original_str, mapping):
        """
        Replaces keys in mapping with the corresponding values
        :param original_str: string in which replacements are to be made
        :param mapping: key, values as to_replace and replace_with_obj(replace_with is the attribute "ros2_name")
        :return:
        """
        for to_replace in mapping:
            if to_replace != Constants.NEW_TOKENS_LIST:
                original_str = original_str.replace(to_replace, mapping[to_replace][Constants.ROS_2_NAME])
        return original_str

    @staticmethod
    def init_ros2_folder():
        """
        Creates a new folder in ROS2_OUTPUT_DIR, with name src_currDate_currTime, and then copies the contents of ROS1
        package from ROS1_SRC_PATH to the created directory
        :return:
        """
        Utilities.ROS2_SRC_PATH = os.path.join(Utilities.ROS2_OUTPUT_DIR, Utilities.get_uniquie_id(),
                                               Constants.ROS2_SRC_FOLDER)
        Utilities.copy_directory(Utilities.ROS1_SRC_PATH, Utilities.ROS2_SRC_PATH)


if __name__ == "__main__":
    Utilities.init()
    Utilities.init_ros2_folder()
    print(Utilities.ROS2_SRC_PATH)