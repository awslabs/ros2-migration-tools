import copy
import datetime
import difflib
import json
import os
import re
import shutil
import sys

from Constants import AstConstants, Constants, MappingConstants, RosConstants


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
            directory = os.path.dirname(full_file_path)
            if not os.path.exists(directory):
                os.makedirs(directory)
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
            original_str = original_str.replace(to_replace, mapping[to_replace][Constants.ROS_2_NAME])
        return original_str

    @staticmethod
    def init_ros2_folder(is_debugging):
        """
        Creates a new folder in ROS2_OUTPUT_DIR, with name src_currDate_currTime, and then copies the contents of ROS1
        package from ROS1_SRC_PATH to the created directory
        :param is_debugging: Flag to check if debugging
        :return:
        """
        folder_name = "debug_ros" if is_debugging else Utilities.get_uniquie_id()
        Utilities.ROS2_SRC_PATH = os.path.join(Utilities.ROS2_OUTPUT_DIR, folder_name, Constants.ROS2_SRC_FOLDER)

        if os.path.exists(Utilities.ROS2_SRC_PATH):
            shutil.rmtree(Utilities.ROS2_SRC_PATH)

        if is_debugging:
            src2 = Utilities.get_parent_dir(Utilities.ROS2_SRC_PATH) + "_backup"
            if os.path.exists(src2):
                shutil.rmtree(src2)
            Utilities.copy_directory(Utilities.ROS1_SRC_PATH, src2)

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

        for token_type in Constants.HELPER_TOKEN_TYPES:
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
            if token_kind not in Constants.HELPER_TOKEN_TYPES and token_kind not in Constants.TOKEN_TYPES:
                continue

            tokens_list = ast_tokens[token_kind]
            for token in tokens_list:
                line = token[AstConstants.LINE]
                if line not in ast_for_line:
                    ast_for_line[line] = Utilities.get_line_by_line_template()

                ast_for_line[line][token_kind].append(token)
        return ast_for_line

    @staticmethod
    def store_ast_line_by_line(tokens_dict):
        """
        Stores ast line by line for each file in tokens_dict and returns the ast in new format
        :param tokens_dict: dict containing list of tokens of various kind
        :return: dict
        """
        ast_for_file = {}
        for token_kind in tokens_dict:
            if token_kind not in Constants.HELPER_TOKEN_TYPES and token_kind not in Constants.TOKEN_TYPES:
                continue

            tokens = tokens_dict[token_kind]
            for token in tokens:
                line = token[AstConstants.LINE]
                src_file_path = token[AstConstants.SRC_FILE_PATH]
                if src_file_path not in ast_for_file:
                    ast_for_file[src_file_path] = {}

                if line not in ast_for_file[src_file_path]:
                    ast_for_file[src_file_path][line] = Utilities.get_line_by_line_template()

                ast_for_file[src_file_path][line][token_kind].append(token)
        return ast_for_file

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
    def get_node_var_parent_class(ast_dict, node_handle_var_name):
        """
        Returns the name of the class containing declaration of `node_handle_var_name`
        :param ast_dict: dict of tokens
        :param node_handle_var_name: variable name for ros::NodeHandle
        :return: str or None
        """
        if AstConstants.CLASS_DECL in ast_dict:
            for token in ast_dict[AstConstants.CLASS_DECL]:
                line_tokens = token[AstConstants.LINE_TOKENS]
                if node_handle_var_name in line_tokens:
                    index = line_tokens.index(node_handle_var_name)

                    # line_token will contain keywords in this order: [..., "ros","::","NodeHandle","node_handle_", ...]
                    if index >= 3:
                        decl_stmt = "".join(line_tokens[index-3:index])
                        if decl_stmt == RosConstants.NODE_HANDLE:
                            return token[AstConstants.NAME]
        return None

    @staticmethod
    def get_ros_node_info(ast_dict):
        """
        Return a dict containing node_name and node_var_name
        :param ast_dict: dict of tokens
        :return: dict
        """
        node_name = Utilities.get_node_name(ast_dict)
        if node_name is None:
            node_name = str(input("Node name not found. Enter Node name: "))

        node_handle_var_name = Utilities.get_node_handle_var_name(ast_dict)
        if node_handle_var_name is None:
            node_handle_var_name = str(input("NodeHandle var not found. Enter NodeHandle var name name: "))

        node_var_parent_class = Utilities.get_node_var_parent_class(ast_dict, node_handle_var_name)

        return {
            Constants.NODE_NAME: node_name,
            Constants.NODE_HANDLE_VAR_NAME: node_handle_var_name,
            Constants.NODE_VAR_PARENT_CLASS: node_var_parent_class
        }

    @staticmethod
    def merge_ast_dict(old_ast_dict, new_ast_dict):
        """
        Merge all the tokens from new_ast_dict to old_ast_dict and return the merged dict
        :param old_ast_dict: dict to which to merge to
        :param new_ast_dict: dict going to be merged to old_ast_dict
        :return: None
        """
        for kind in new_ast_dict:
            if kind in old_ast_dict:
                old_ast_dict[kind].extend(new_ast_dict[kind])
            else:
                old_ast_dict[kind] = new_ast_dict[kind]

    @staticmethod
    def merge_ast_line_dict(old_ast_dict, new_ast_dict):
        """
        Merge all the tokens from new_ast_dict to old_ast_dict and return the merged dict
        :param old_ast_dict: dict to which to merge to
        :param new_ast_dict: dict going to be merged to old_ast_dict
        :return: None
        """
        for file_path in new_ast_dict:
            if file_path not in old_ast_dict:
                old_ast_dict[file_path] = new_ast_dict[file_path]
            else:
                for line_number in new_ast_dict[file_path]:
                    if line_number not in old_ast_dict[file_path]:
                        old_ast_dict[file_path][line_number] = new_ast_dict[file_path][line_number]
                    else:
                        Utilities.merge_ast_dict(old_ast_dict[file_path][line_number], new_ast_dict[file_path][line_number])

    @staticmethod
    def print_json(obj):
        """
        To print json with indentation
        :param obj:
        :return: None
        """
        print(json.dumps(obj, indent=4))

    @staticmethod
    def replace_word_in_line(line, to_replace, replace_with):
        """
        In `line`, replace `replace_token` with `replace_with`, and return the new line
        :param line: line in which replacement is to be done
        :param to_replace: candidate to replace
        :param replace_with: replace with
        :return: str
        """
        pattern = "\\b" + to_replace + "\\b"
        return re.sub(pattern, replace_with, line)

    @staticmethod
    def get_diff_content_of_files(src_content, new_src, src_file):
        """
        Returns a list containing the diff between the `src_content` and `new_src`
        :param src_content: old content of the file
        :param new_src: new content after migration
        :param src_file: file path
        :return: list
        """
        lines1 = src_content.splitlines()
        lines2 = new_src.splitlines()

        diff_content = []

        deletions = []
        additions = []
        for line in difflib.ndiff(lines1, lines2):
            if line[0] == '-':
                deletions.append(line)
            if line[0] == '+':
                additions.append(line)

        diff_content.append('#' * 10 + src_file + '#' * 10)
        diff_content.append('=' * 160)

        for i, line in enumerate(deletions):
            diff_content.append(line)
            if i < len(additions):
                diff_content.append(additions[i])
            diff_content.append('='*160)

        diff_content.append("\n\n")
        return diff_content

    @staticmethod
    def is_unit_test_path(path):
        """
        Returns True if the token is from unit test file
        :param path: file path
        :return: boolean
        """
        # Assuming all unit test will follow path like `src/some_folder1/some_folder2/test/reader_test.cpp`
        # where 'test' occurs at least twice
        if path.count('test') > 1:
            return True

        # check if 'gtest' is part of path, then also exclude it
        if "gtest" in path:
            return True

    @staticmethod
    def get_all_files_of_extension(directory, file_extensions):
        """
        Returns a list of files from `directory` and its sub-directories which have one of the file extension from
        `file_extensions`
        :param directory: directory in which to look for files
        :param file_extensions: list of file extensions
        :return: list
        """
        file_list = []
        for root, dirs, files in os.walk(directory):
            for file in files:
                for ext in file_extensions:
                    if file.endswith(ext):
                        file_list.append(os.path.join(root, file))

        return file_list

    @staticmethod
    def get_consolidated_mapping():
        """
        Reads all the valid .json files in the `mapping` folder and created a consolidated dict
        :return: dict
        """
        file_list = Utilities.get_all_files_of_extension(MappingConstants.MAPPING_FOLDER, ".json")

        consolidated_mapping = Utilities.read_json_file(os.path.join(MappingConstants.MAPPING_FOLDER,
                                                                     MappingConstants.MASTER_MAPPING_FILE_NAME))
        for file_path in file_list:
            is_valid_file = True
            for ignore_file in MappingConstants.IGNORE_LIST:
                if file_path.endswith(ignore_file) or file_path.endswith(MappingConstants.MASTER_MAPPING_FILE_NAME):
                    is_valid_file = False
                    break

            if is_valid_file:
                curr_mapping = Utilities.read_json_file(file_path)
                for token_kind in consolidated_mapping:
                    for token_name in curr_mapping[token_kind]:
                        consolidated_mapping[token_kind][token_name] = curr_mapping[token_kind][token_name]

        return consolidated_mapping

    @staticmethod
    def get_consolidated_token_filters():
        """
        Reads all the .json files in the `token_filter` folder and created a consolidated list of `IRRELEVANT_TOKENS`
        :return: list
        """
        file_list = Utilities.get_all_files_of_extension(MappingConstants.TOKEN_FILTERS_FOLDER, ".json")

        consolidated_filter_tokens = []
        for file_path in file_list:
            curr_tokens = Utilities.read_json_file(file_path)
            consolidated_filter_tokens.extend(curr_tokens[MappingConstants.IRRELEVANT_TOKENS])

        return consolidated_filter_tokens

    @staticmethod
    def get_new_tokens_template():
        """
        Returns a template for new tokens file
        :return: dict
        """
        new_token_template = {}
        for token_type in Constants.TOKEN_TYPES:
            new_token_template[token_type] = []

        return new_token_template


if __name__ == "__main__":
    is_debugging = False
    if len(sys.argv) == 2 and sys.argv[1] == Constants.DEBUGGING:
        is_debugging = True
    Utilities.init()
    Utilities.init_ros2_folder(is_debugging)
    print(Utilities.ROS2_SRC_PATH)