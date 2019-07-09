from bonsai.cpp.clang_parser import CppAstParser
from porting_tools.cmake_lists_porter import CMakeListsPorter
from porting_tools.package_xml_porter import PackageXMLPorter
from porting_tools.source_code_porter import SourceCodePorter
from Constants import AstConstants, Constants
from utilities import Utilities
import xml.etree.ElementTree as etree
import os
import json
import fnmatch
import sys
import copy

class RosUpgrader:
    """
    Convert ROS1 package to ROS2 package
    """
    TOKEN_TYPES = (
        AstConstants.CALL_EXPR,
        AstConstants.NAMESPACE_REF,
        AstConstants.VAR_DECL,
        AstConstants.MACRO_INSTANTIATION,
        AstConstants.INCLUSION_DIRECTIVE
    )

    MANUAL_TOKENS = (
        Constants.INCLUDES
    )

    DECLARATION_PATH_CHECK = {
        AstConstants.CALL_EXPR: True,
        AstConstants.NAMESPACE_REF: True,
        AstConstants.VAR_DECL: False,
        AstConstants.MACRO_INSTANTIATION: True,
        AstConstants.INCLUSION_DIRECTIVE: False
    }

    # path to the folder containing "compile_commands.json" file, will be generated using cmake flag
    # "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    COMPILE_JSON_PATH = None

    # folder to upgrade
    SRC_PATH_TO_UPGRADE = None

    # path to all the include folders required by the package
    INCLUDES = []

    # AST dict. Keys will be line numbers. Values will be dict with TOKEN_TYPES as keys and values will be the tokens
    AST_LINE_BY_LINE = None

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

        CppAstParser.set_library_path(Utilities.get_abs_path(config_obj[Constants.LIBCLANG_PATH]))

        RosUpgrader.INCLUDES = []
        for fpath in config_obj[Constants.USER_INCLUDES]:
            RosUpgrader.INCLUDES.append(Utilities.get_abs_path(fpath))

    @staticmethod
    def get_cpp_file_list():
        """
        Reads the "compile_commands.json" file from COMPILE_DB_DIR and get the list of .cpp files, which will be the
        entry points for parsing
        :return: list containing full path of all the .cpp files
        """
        file_list = []

        entries = Utilities.read_json_file(RosUpgrader.COMPILE_JSON_PATH)
        for entry in entries:
            if ".cpp" in entry["file"]:
                file_list.append(entry["file"])

        return file_list

    @staticmethod
    def get_ast_as_json():
        """
        Creates an instance of CppAstParser and gets all the tokens for various .cpp files as a python dict
        :return: dict with key as path to .cpp and values as the tokens dict
        """
        cp = CppAstParser(RosUpgrader.SRC_PATH_TO_UPGRADE, RosUpgrader.INCLUDES)

        file_list = RosUpgrader.get_cpp_file_list()

        all_tokens = {}
        for file_path in file_list:
            all_tokens[file_path] = cp.get_ast_obj(file_path)

        return all_tokens

    @staticmethod
    def get_all_file_paths(directory, file_name):
        """
        Recursively find all the file with name 'file_name' in "directory"
        :param file_name: name of the file to search recursively
        :return: list containing file paths
        """
        matches = []
        for root, dirnames, filenames in os.walk(directory):
            for file in fnmatch.filter(filenames, file_name):
                matches.append(os.path.join(root, file))
        return matches

    @staticmethod
    def convert_all_cmake():
        """
        Converts all the cmake files from ROS1 to ROS2
        :return: None
        """
        cmake_files_list = RosUpgrader.get_all_file_paths(RosUpgrader.SRC_PATH_TO_UPGRADE, Constants.CMAKE_FILE_NAME)

        for file_path in cmake_files_list:
            src_file = Utilities.read_from_file(file_path)
            ported_content = CMakeListsPorter.port(content=src_file)
            Utilities.write_to_file(file_path, ported_content)

    @staticmethod
    def convert_all_package_xml():
        """
        Converts all package.xml files from ROS1 to ROS2
        :return: None
        """
        package_files_list = RosUpgrader.get_all_file_paths(RosUpgrader.SRC_PATH_TO_UPGRADE, Constants.PACKAGE_XML_FILE)

        for file_path in package_files_list:
            tree = etree.parse(file_path)
            PackageXMLPorter.port(tree=tree)
            tree.write(file_path, encoding="utf-8", xml_declaration=True)

    @staticmethod
    def get_declared_functions(tokens):
        """
        Creates a list containing all the tokens of kind FUNCTION_DECL
        :param tokens:
        :return:
        """
        declared_func_list = set()
        if AstConstants.FUNCTION_DECL in tokens:
            for token in tokens[AstConstants.FUNCTION_DECL]:
                declared_func_list.add(token[AstConstants.NAME])

        return declared_func_list

    @staticmethod
    def get_mapping_attributes(token):
        """
        Populates attributes for the token to be saved in NEW_TOKEN_LIST
        :param token:
        :return: dict containing the attributes
        """
        attributes = {
            Constants.ROS_1_NAME: token[AstConstants.NAME],
            Constants.ROS_2_NAME: Constants.NEW_MAPPING_MSG
        }
        if token[AstConstants.KIND] == AstConstants.VAR_DECL:
            attributes[Constants.ROS_1_NAME] = token[AstConstants.VAR_TYPE]
            attributes[Constants.TO_SHARED_PTR] = True
            attributes[Constants.CREATION_INFO] = {
                Constants.IS_CREATED_BY_NODE: False,
                Constants.MEMBER_NAME_IF_TRUE: ""
            }
        elif token[AstConstants.KIND] == AstConstants.CALL_EXPR:
            attributes[Constants.NODE_ARG_INFO] = {
                Constants.NODE_ARG_REQ: False,
                Constants.NODE_ARG_INDEX: -1
            }
        elif token[AstConstants.KIND] == AstConstants.MACRO_INSTANTIATION:
            attributes[Constants.NODE_ARG_INFO] = {
                Constants.NODE_ARG_REQ: False,
                Constants.NODE_ARG_INDEX: -1,
                Constants.MEMBER_NAME_IF_TRUE: ""
            }
        return attributes

    @staticmethod
    def check_token_validity(token, mappings, irrelevant_tokens, added_set):
        """
        Checks for validity of a token
        :param token:
        :param mappings: mappings for token type
        :param irrelevant_tokens: set of tokens to ignore
        :param added_set: set of tokens already added
        :return: True if token is valid False otherwise
        """
        if token[AstConstants.NAME] == AstConstants.NO_SPELLING:
            return False

        if RosUpgrader.DECLARATION_PATH_CHECK[token[AstConstants.KIND]]:
            if Constants.ROS1_INCLUDE_PATH not in token[AstConstants.DECL_FILEPATH]:
                return False

        token_identifier = token[AstConstants.NAME]

        # checking if there is already a duplicate mapping present
        if token[AstConstants.KIND] == AstConstants.VAR_DECL:
            token_identifier = token[AstConstants.VAR_TYPE]
            if token[AstConstants.VAR_TYPE] in mappings:
                return False
        else:
            if token[AstConstants.NAME] in mappings:
                return False

        if token_identifier in irrelevant_tokens or token_identifier in added_set:
            return False

        return True

    @staticmethod
    def get_line_by_line_template():
        """
        Generates a template for token types to be stored line by line
        :return: dict
        """
        template = {}
        for token_type in RosUpgrader.TOKEN_TYPES:
            template[token_type] = []

        return copy.deepcopy(template)

    @staticmethod
    def store_ast_line_by_line(tokens_dict):
        """
        Stores ast line by line for each file in tokens_dict
        :param tokens_dict:
        :return: None
        """
        RosUpgrader.AST_LINE_BY_LINE = {}
        for file in tokens_dict:
            RosUpgrader.AST_LINE_BY_LINE[file] = {}
            ast_for_file = RosUpgrader.AST_LINE_BY_LINE[file]
            for token_kind in tokens_dict[file]:
                if token_kind not in RosUpgrader.TOKEN_TYPES:
                    continue

                tokens = tokens_dict[file][token_kind]
                for token in tokens:
                    line = token[AstConstants.LINE]
                    if line not in ast_for_file:
                        ast_for_file[line] = RosUpgrader.get_line_by_line_template()

                    ast_for_file[line][token_kind].append(token)

    @staticmethod
    def add_new_mappings():
        """
        Parses the file and adds the mappings to "mappings.json" if the mapping was missing
        :return: None
        """
        mappings = Utilities.read_json_file(Constants.MAPPING_FILE_NAME)

        irrelevant_tokens = set(mappings[Constants.IRRELEVANT_TOKENS])

        tokens_dict = RosUpgrader.get_ast_as_json()
        RosUpgrader.store_ast_line_by_line(tokens_dict)

        for file in tokens_dict:
            tokens = tokens_dict[file]
            for token_type in RosUpgrader.TOKEN_TYPES:
                if token_type in tokens:
                    added_set = set()
                    for token in tokens[token_type]:
                        if RosUpgrader.check_token_validity(token, mappings[token_type], irrelevant_tokens, added_set):

                            mappings[token_type][Constants.NEW_TOKENS_LIST].append(
                                RosUpgrader.get_mapping_attributes(token))

                            added_set.add(token[AstConstants.NAME])
                        else:
                            if token_type == AstConstants.USING_DIRECTIVE:
                                pass

        Utilities.write_as_json(Constants.MAPPING_FILE_NAME, mappings)

    @staticmethod
    def convert_all_source_files():
        """
        Calls the porter on all source files to convert the ROS1 source to ROS2
        :return: None
        """
        mapping = Utilities.read_json_file(Constants.MAPPING_FILE_NAME)
        if mapping is not None:
            file_list = RosUpgrader.get_cpp_file_list()
            for file_path in file_list:
                src_content = Utilities.read_from_file(file_path)
                new_src = SourceCodePorter.port(source=src_content, mapping=mapping,
                                                ast=RosUpgrader.AST_LINE_BY_LINE[file_path])
                Utilities.write_to_file(file_path, new_src)

    @staticmethod
    def add_filled_mappings():
        """
        Adds the filled mapping from NEW_TOKENS_LIST to the node type
        :return: None
        """
        mapping = Utilities.read_json_file(Constants.MAPPING_FILE_NAME)
        for token_type in mapping:
            if token_type != Constants.IRRELEVANT_TOKENS:
                new_token_list = mapping[token_type][Constants.NEW_TOKENS_LIST]
                for new_token in new_token_list:
                    if new_token[Constants.ROS_2_NAME] != Constants.NEW_MAPPING_MSG:
                        updated_token = copy.deepcopy(new_token)
                        del updated_token[Constants.ROS_1_NAME]
                        mapping[token_type][new_token[Constants.ROS_1_NAME]] = updated_token
                    else:
                        mapping[Constants.IRRELEVANT_TOKENS].append(new_token[Constants.ROS_1_NAME])
                mapping[token_type][Constants.NEW_TOKENS_LIST] = []
        Utilities.write_as_json(Constants.MAPPING_FILE_NAME, mapping)

    @staticmethod
    def start_upgrade():
        """
        Calls functions required for the upgrade process
        :return: None
        """
        RosUpgrader.convert_all_cmake()
        RosUpgrader.convert_all_package_xml()
        RosUpgrader.add_new_mappings()

        prompt = str(input("All mappings filled? Press 'Y' to continue or any other key to abort"))
        if prompt == 'Y':
            RosUpgrader.add_filled_mappings()
            RosUpgrader.convert_all_source_files()
        else:
            print("Aborting...")

    @staticmethod
    def set_compile_db(compile_db_dir):
        """
        Initializes CppAstParser compile database
        :param compile_db_dir:
        :return:
        """
        CppAstParser.set_database(Utilities.get_abs_path(compile_db_dir))

    def __init__(self):
        pass


def main():
    if len(sys.argv) != 2:
        raise Exception("ros_upgrader.py needs SRC_PATH_TO_UPGRADE as argument")
    RosUpgrader.SRC_PATH_TO_UPGRADE = sys.argv[1]

    RosUpgrader.init()

    src_parent_dir = Utilities.get_parent_dir(RosUpgrader.SRC_PATH_TO_UPGRADE)

    compile_db_files = RosUpgrader.get_all_file_paths(src_parent_dir, Constants.COMPILE_COMMANDS_FILE)

    print(compile_db_files)

    for compile_json in compile_db_files:
        RosUpgrader.COMPILE_JSON_PATH = compile_json
        RosUpgrader.set_compile_db(Utilities.get_parent_dir(compile_json))
        RosUpgrader.start_upgrade()


if __name__ == '__main__':
    main()






