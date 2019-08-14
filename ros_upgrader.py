import copy
import fnmatch
import json
import logging
import os
import shutil
import sys
import xml.etree.ElementTree as etree

from clang.clang_parser import CppAstParser

from Constants import AstConstants, Constants, MappingConstants
from porting_tools.cmake_lists_porter import CMakeListsPorter
from porting_tools.package_xml_porter import PackageXMLPorter
from porting_tools.cpp_source_code_porter import CPPSourceCodePorter
from utilities import Utilities

logging.basicConfig(filename='logs/ros_upgrader' + Utilities.get_uniquie_id() + '.log',
                    filemode='w',
                    format='%(name)s - %(levelname)s - %(message)s',
                    level=logging.DEBUG)


class RosUpgrader:
    """
    Convert ROS1 package to ROS2 package
    """

    DECLARATION_PATH_CHECK = {
        AstConstants.CALL_EXPR: True,
        AstConstants.NAMESPACE_REF: True,
        AstConstants.VAR_DECL: False,
        AstConstants.MACRO_INSTANTIATION: True,
        AstConstants.INCLUSION_DIRECTIVE: False,
        AstConstants.PARM_DECL: False,
        AstConstants.CONVERSION_FUNCTION: False
    }

    # path to the folder containing "compile_commands.json" file, will be generated using cmake flag
    # "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    COMPILE_JSON_PATH = None

    # folder to upgrade
    SRC_PATH_TO_UPGRADE = None

    # AST dict. Keys will be line numbers. Values will be dict with TOKEN_TYPES as keys and values will be the tokens
    AST_LINE_BY_LINE = {}

    AST_DICT = {
        AstConstants.NON_UNIT_TEST: {},
        AstConstants.UNIT_TEST: {}
    }

    # dict containing list of filtered out token ids
    irrelevant_tokens = Utilities.read_json_file(os.path.join(MappingConstants.MAPPING_FOLDER,
                                                              MappingConstants.FILTERED_OUT_FILE_NAME))

    # dict containing consolidated mappings of all valid .json files in `mappings` folder
    consolidated_mapping = Utilities.get_consolidated_mapping()

    # flag to check if new mapping has been added to `new_tokens.json`
    new_mappings_added = False

    # template file for filled mappings
    mapping_template = Utilities.read_json_file(os.path.join(MappingConstants.MAPPING_FOLDER,
                                                             MappingConstants.MAPPING_TEMPLATE_FILE_NAME))

    @staticmethod
    def init():
        """
        Initializes CppAstParser and Clang. Stores various config paths to be used later
        :return: None
        """
        try:
            config = open(Constants.CONFIG_FILE_NAME)
        except IOError as e:
            raise e

        config_obj = json.load(config)

        CppAstParser.set_library_path(Utilities.get_abs_path(config_obj[Constants.LIBCLANG_PATH]))
        CppAstParser.set_standard_includes(os.path.join(Utilities.get_abs_path(config_obj[Constants.LIBCLANG_PATH]),
                                                        "include"))

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
    def get_ast_as_json(compile_db_path):
        """
        Creates an instance of CppAstParser and gets all the tokens for various .cpp files as a python dict
        :param compile_db_path: path to compile_commands.json file
        :return: dict with key as path to .cpp and values as the tokens dict
        """
        cp = CppAstParser(compile_db_path, RosUpgrader.SRC_PATH_TO_UPGRADE)
        all_tokens = cp.get_ast_obj()

        return all_tokens

    @staticmethod
    def get_all_file_paths(directory, file_name):
        """
        Recursively find all the file with name 'file_name' in "directory"
        :param directory: directory in which to search
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
    def get_mapping_attributes(token):
        """
        Populates attributes for the token to be saved in NEW_TOKEN_LIST
        :param token:
        :return: dict containing the attributes
        """
        attributes = {
            Constants.ROS_1_NAME: token[AstConstants.NAME],
            Constants.ROS_2_NAME: MappingConstants.NEW_MAPPING_MSG,
            Constants.TOKEN_LOCATION: {
                AstConstants.SRC_FILE_PATH: token[AstConstants.SRC_FILE_PATH],
                AstConstants.LINE: token[AstConstants.LINE],
                AstConstants.TOKEN_START_COL: token[AstConstants.TOKEN_START_COL],
                AstConstants.TOKEN_END_COL: token[AstConstants.TOKEN_END_COL]
            }
        }

        if token[AstConstants.KIND] == AstConstants.VAR_DECL or token[AstConstants.KIND] == AstConstants.PARM_DECL:
            attributes[Constants.ROS_1_NAME] = token[AstConstants.VAR_TYPE]
            attributes[Constants.TO_BE_REMOVED] = False
            if token[AstConstants.KIND] == AstConstants.VAR_DECL:
                attributes[Constants.TO_SHARED_PTR] = False
                attributes[Constants.NODE_ARG_INFO] = {
                    Constants.NODE_ARG_REQ: False,
                    Constants.NODE_ARG_INDEX: -1
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
        if token[AstConstants.KIND] == AstConstants.VAR_DECL or token[AstConstants.KIND] == AstConstants.PARM_DECL:
            token_identifier = token[AstConstants.VAR_TYPE]
            if token[AstConstants.VAR_TYPE] in mappings:
                return False
        else:
            if token[AstConstants.NAME] in mappings:
                return False

        irrelevant_token_id = token_identifier + "_" + token[AstConstants.KIND]
        if irrelevant_token_id in irrelevant_tokens or token_identifier in added_set:
            return False

        return True

    @staticmethod
    def add_new_mappings(compile_db_path, new_mappings, already_added_mappings):
        """
        Parses the file and adds the mappings to "master_mappings.json" if the mapping was missing
        :param compile_db_path: path to compile_json_file
        :param new_mappings: dict containing ROS1->ROS2 mapping for various token kinds
        :param already_added_mappings: dict containing list of token identifiers for various kind which has already been
        added as new mapping
        :return: None
        """
        ast_dict = RosUpgrader.get_ast_as_json(compile_db_path)
        Utilities.merge_ast_dict(RosUpgrader.AST_DICT[AstConstants.NON_UNIT_TEST], ast_dict[AstConstants.NON_UNIT_TEST])
        Utilities.merge_ast_dict(RosUpgrader.AST_DICT[AstConstants.UNIT_TEST], ast_dict[AstConstants.UNIT_TEST])

        Utilities.merge_ast_line_dict(RosUpgrader.AST_LINE_BY_LINE,
                                      Utilities.store_ast_line_by_line(RosUpgrader.AST_DICT[AstConstants.NON_UNIT_TEST]))
        Utilities.merge_ast_line_dict(RosUpgrader.AST_LINE_BY_LINE,
                                      Utilities.store_ast_line_by_line(RosUpgrader.AST_DICT[AstConstants.UNIT_TEST]))

        irrelevant_tokens = set(RosUpgrader.irrelevant_tokens[MappingConstants.IRRELEVANT_TOKENS])

        for token_type in Constants.TOKEN_TYPES:
            for ast_category in RosUpgrader.AST_DICT:
                if token_type in RosUpgrader.AST_DICT[ast_category]:
                    if token_type not in already_added_mappings:
                        already_added_mappings[token_type] = set()
                    added_set = already_added_mappings[token_type]
                    for token in RosUpgrader.AST_DICT[ast_category][token_type]:
                        if RosUpgrader.check_token_validity(token,
                                                            RosUpgrader.consolidated_mapping[token_type],
                                                            irrelevant_tokens,
                                                            added_set):

                            new_mappings[token_type].append(RosUpgrader.get_mapping_attributes(token))

                            if token[AstConstants.KIND] == AstConstants.VAR_DECL or \
                                    token[AstConstants.KIND] == AstConstants.PARM_DECL:
                                added_set.add(token[AstConstants.VAR_TYPE])
                            else:
                                added_set.add(token[AstConstants.NAME])
                        else:
                            if token_type == AstConstants.USING_DIRECTIVE:
                                pass

    @staticmethod
    def convert_all_source_files():
        """
        Calls the porter on all source files to convert the ROS1 source to ROS2
        :return: None
        """
        mapping = RosUpgrader.consolidated_mapping
        if mapping is not None:
            file_list = Utilities.get_all_files_of_extension(RosUpgrader.SRC_PATH_TO_UPGRADE, [".cpp", ".h", ".hpp"])

            diff_content = []
            for file_path in file_list:
                src_content = Utilities.read_from_file(file_path)

                cpp_porter = CPPSourceCodePorter(RosUpgrader.AST_DICT, file_path)

                if file_path in RosUpgrader.AST_LINE_BY_LINE:
                    new_src = cpp_porter.port(source=src_content,
                                              mapping=RosUpgrader.consolidated_mapping,
                                              ast=RosUpgrader.AST_LINE_BY_LINE[file_path])

                    Utilities.write_to_file(file_path, new_src)

                    diff_content.extend(Utilities.get_diff_content_of_files(src_content, new_src, file_path))

            Utilities.write_to_file(os.path.join(Utilities.get_parent_dir(RosUpgrader.SRC_PATH_TO_UPGRADE), Constants.DIFF_FILE_PATH), "\n".join(diff_content))

    @staticmethod
    def add_filled_mappings(mapping_file_name=MappingConstants.MASTER_MAPPING_FILE_NAME):
        """
        Adds the filled mapping from NEW_TOKENS_FILE to the node type
        :param mapping_file_name: file to write the filled mappings to, if file doesn't exist then new one will be created
        :return: None
        """
        mapping_file_path = os.path.join(MappingConstants.MAPPING_FOLDER, mapping_file_name)
        new_mappings = Utilities.read_json_file(os.path.join(MappingConstants.MAPPING_FOLDER,
                                                             MappingConstants.NEW_TOKENS_FILE_NAME))

        if os.path.exists(mapping_file_path):
            mapping = Utilities.read_json_file(mapping_file_path)
        else:
            mapping = copy.deepcopy(RosUpgrader.mapping_template)

        for token_type in mapping:
            new_token_list = new_mappings[token_type]
            for new_token in new_token_list:
                if new_token[Constants.ROS_2_NAME] != MappingConstants.NEW_MAPPING_MSG:
                    updated_token = copy.deepcopy(new_token)
                    del updated_token[Constants.ROS_1_NAME]
                    del updated_token[Constants.TOKEN_LOCATION]

                    mapping[token_type][new_token[Constants.ROS_1_NAME]] = updated_token
                else:
                    irrelevant_token_id = new_token[Constants.ROS_1_NAME] + "_" + token_type
                    RosUpgrader.irrelevant_tokens[MappingConstants.IRRELEVANT_TOKENS].append(irrelevant_token_id)
            new_mappings[token_type] = []

        Utilities.write_as_json(mapping_file_path, mapping)
        Utilities.write_as_json(os.path.join(MappingConstants.MAPPING_FOLDER, MappingConstants.FILTERED_OUT_FILE_NAME),
                                RosUpgrader.irrelevant_tokens)
        Utilities.write_as_json(os.path.join(MappingConstants.MAPPING_FOLDER, MappingConstants.NEW_TOKENS_FILE_NAME), {})

    @staticmethod
    def check_if_new_mappings_added(new_mappings):
        for kind in new_mappings:
            if len(new_mappings[kind]):
                return True
        return False

    @staticmethod
    def start_upgrade(new_mappings):
        """
        Calls functions required for the upgrade process
        :param new_mappings: new mappings json to be filled by user
        :return: None
        """
        RosUpgrader.convert_all_cmake()
        RosUpgrader.convert_all_package_xml()

        RosUpgrader.new_mappings_added = RosUpgrader.check_if_new_mappings_added(new_mappings)

        if RosUpgrader.new_mappings_added:
            Utilities.write_as_json(os.path.join(MappingConstants.MAPPING_FOLDER, MappingConstants.NEW_TOKENS_FILE_NAME),
                                    new_mappings)
            prompt = str(input("All mappings filled? Press 'Y' to continue or any other key to abort: "))
            if prompt == 'Y':
                RosUpgrader.add_filled_mappings()
                RosUpgrader.convert_all_source_files()
            else:
                print("Aborting...")
        else:
            RosUpgrader.convert_all_source_files()

    def __init__(self):
        pass


def is_debugging():
    if len(sys.argv) == 3 and sys.argv[2] == Constants.DEBUGGING:
        return True
    return False


def main():
    if len(sys.argv) < 2:
        raise Exception("ros_upgrader.py needs SRC_PATH_TO_UPGRADE as argument")

    RosUpgrader.SRC_PATH_TO_UPGRADE = sys.argv[1]

    if is_debugging():
        shutil.rmtree(RosUpgrader.SRC_PATH_TO_UPGRADE)
        Utilities.copy_directory(Utilities.get_parent_dir(RosUpgrader.SRC_PATH_TO_UPGRADE) + "_backup",
                                 RosUpgrader.SRC_PATH_TO_UPGRADE)

    RosUpgrader.init()

    src_parent_dir = Utilities.get_parent_dir(RosUpgrader.SRC_PATH_TO_UPGRADE)

    compile_db_files = RosUpgrader.get_all_file_paths(src_parent_dir, Constants.COMPILE_COMMANDS_FILE)

    new_mappings = Utilities.get_new_tokens_template()

    already_added_mappings = {}
    for compile_json in compile_db_files:
        RosUpgrader.COMPILE_JSON_PATH = compile_json
        RosUpgrader.add_new_mappings(Utilities.get_parent_dir(compile_json), new_mappings, already_added_mappings)

    if is_debugging():
        Utilities.write_as_json("ast_dump_" + AstConstants.NON_UNIT_TEST + ".json",
                                RosUpgrader.AST_DICT[AstConstants.NON_UNIT_TEST])
        Utilities.write_as_json("ast_dump_" + AstConstants.UNIT_TEST + ".json",
                                RosUpgrader.AST_DICT[AstConstants.UNIT_TEST])
    RosUpgrader.start_upgrade(new_mappings)


if __name__ == '__main__':
    main()
