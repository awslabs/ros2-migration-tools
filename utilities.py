import copy
import datetime
import json
import os
import shutil

from Constants import AstConstants, Constants, RosConstants


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

    @staticmethod
    def get_line_by_line_template():
        """
        Generates a template for token types to be stored line by line
        :return: dict
        """
        template = {}
        for token_type in Constants.TOKEN_TYPES:
            template[token_type] = []

        return copy.deepcopy(template)

    @staticmethod
    def get_line_by_line_ast(ast_tokens):
        """
        Retuns a dict containing array of tokens for each line
        :param ast_tokens: dict containing tokens list for each kind
        :return: dict
        """
        ast_for_line = {}
        for token_kind in ast_tokens:
            if token_kind not in Constants.TOKEN_TYPES:
                continue

            tokens_list = ast_tokens[token_kind]
            for token in tokens_list:
                line = token[AstConstants.LINE]
                if line not in ast_for_line:
                    ast_for_line[line] = Utilities.get_line_by_line_template()

                ast_for_line[line][token_kind].append(token)
        return ast_for_line

    @staticmethod
    def get_raw_str(input_str):
        """
        Returns the raw string representation of input_str
        :param input_str: input string
        :return: str
        """
        return repr(input_str)[1:-1]

    @staticmethod
    def get_node_name(ast_dict):
        """
        Returns the name of node if found in ast_dict otherwise returns None
        :param ast_dict: dict of tokens
        :return: string or None
        """
        node_name = None
        if AstConstants.CALL_EXPR in ast_dict:
            for token in ast_dict[AstConstants.CALL_EXPR]:
                if token[AstConstants.NAME] == RosConstants.INIT_CALL_EXPR and \
                        Constants.ROS1_INCLUDE_PATH in token[AstConstants.DECL_FILEPATH]:

                    # ros::init(argc, argv, "talker")
                    line_tokens = token[AstConstants.LINE_TOKENS]
                    if len(line_tokens) == RosConstants.ROS_INIT_LINE_TOKEN_LENGTH:
                        node_name = line_tokens[-2]
                        break

        return node_name

    @staticmethod
    def get_node_handle_var_name(ast_dict):
        """
        Returns name of the NodeHandle var name if found in ast_dict, otherwise return None
        :param ast_dict: dict of tokens
        :return: string or None
        """
        node_handle_var_name = None

        # first try to find in FIELD_DECL
        if AstConstants.FIELD_DECL in ast_dict:
            for token in ast_dict[AstConstants.FIELD_DECL]:
                if token[AstConstants.VAR_TYPE] == RosConstants.NODE_HANDLE:
                    return token[AstConstants.NAME]

        # if not found, then try to find in VAR_DECL
        if AstConstants.VAR_DECL in ast_dict:
            for token in ast_dict[AstConstants.VAR_DECL]:
                if token[AstConstants.VAR_TYPE] == RosConstants.NODE_HANDLE:
                    return token[AstConstants.NAME]

        return node_handle_var_name

    @staticmethod
    def get_ros_node_info(ast_dict):
        """
        Return a dict containing node_name and node_var_name
        :param ast_dict: dict of tokens
        :return: Dict
        """
        node_name = Utilities.get_node_name(ast_dict)
        if node_name is None:
            node_name = str(input("Node name not found. Enter Node name: "))

        node_handle_var_name = Utilities.get_node_handle_var_name(ast_dict)
        if node_handle_var_name is None:
            node_handle_var_name = str(input("NodeHandle var not found. Enter NodeHandle var name name: "))

        return {
            Constants.NODE_NAME: node_name,
            Constants.NODE_HANDLE_VAR_NAME: node_handle_var_name
        }

    @staticmethod
    def get_list_of_pointer_var(ast_dict, mapping):
        """
        Returns a list of var names which are of type shared_ptr
        :param ast_dict: dict of tokens
        :param mapping: ROS1 to ROS2 mapping dict
        :return:
        """

        # get list of var types which are shared_ptr
        var_types = []
        for var_type in mapping[AstConstants.VAR_DECL]:
            if var_type != Constants.NEW_TOKENS_LIST:
                var_info = mapping[AstConstants.VAR_DECL][var_type]
                if var_info[Constants.TO_SHARED_PTR] is True:
                    var_types.append(var_type)

        for var_type in mapping[AstConstants.FIELD_DECL]:
            if var_type != Constants.NEW_TOKENS_LIST:
                var_info = mapping[AstConstants.FIELD_DECL][var_type]
                if var_info[Constants.TO_SHARED_PTR] is True:
                    var_types.append(var_type)

        # now from the ast get the names of the variables which are of type from var_types
        pointer_var_names = []
        if AstConstants.VAR_DECL in ast_dict:
            for token in ast_dict[AstConstants.VAR_DECL]:
                if token[AstConstants.VAR_TYPE] in var_types:
                    pointer_var_names.append(token[AstConstants.NAME])

        if AstConstants.FIELD_DECL in ast_dict:
            for token in ast_dict[AstConstants.FIELD_DECL]:
                if token[AstConstants.VAR_TYPE] in var_types:
                    pointer_var_names.append(token[AstConstants.NAME])

        return pointer_var_names


if __name__ == "__main__":
    Utilities.init()
    Utilities.init_ros2_folder()
    print(Utilities.ROS2_SRC_PATH)