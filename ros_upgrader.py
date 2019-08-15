import argparse
import copy
import fnmatch
import logging
import os
import xml.etree.ElementTree as etree

from clang.clang_parser import CppAstParser

from Constants import AstConstants, Constants, MappingConstants
from porting_tools.cmake_lists_porter import CMakeListsPorter
from porting_tools.package_xml_porter import PackageXMLPorter
from porting_tools.cpp_source_code_porter import CPPSourceCodePorter
from utilities import Utilities

parser = argparse.ArgumentParser()

parser.add_argument('-c', '--compile_db_paths', nargs='+',
                    help='Path to folder containing compile_commands.json file. Can specify multiple paths',
                    required=True)

parser.add_argument('-s', '--src_path', help='Path to the ROS1 package to port', required=True)
parser.add_argument('-o', '--output_path', help='Output directory where the ported files will exist', default='output')
parser.add_argument('-m', '--mapping_file', help='Mapping file to which filled mappings will be added',
                    default=MappingConstants.MASTER_MAPPING_FILE_NAME)

parser.add_argument('-d', '--debug', help='Debug Mode. AST files will be dumped.', action='store_true')

args = parser.parse_args()


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
    COMPILE_JSON_LIST = args.compile_db_paths

    # folder to upgrade
    SRC_PATH_TO_UPGRADE = args.src_path

    # mappings file name to which filled mappings will be added
    MAPPING_FILE_NAME = args.mapping_file

    # output folder for the ported files
    OUTPUT_PATH = None

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

    # list containing src files to be migrated
    src_file_list = []

    @staticmethod
    def init():
        """
        Initializes CppAstParser and Clang. Stores various config paths to be used later
        :return: None
        """
        current_run_id = Utilities.get_uniquie_id()
        logging.basicConfig(filename=os.path.join('logs', 'ros_upgrader_' + current_run_id + '.log'),
                            filemode='w',
                            format='%(name)s - %(levelname)s - %(message)s',
                            level=logging.DEBUG)

        RosUpgrader.OUTPUT_PATH = os.path.join(Utilities.get_abs_path(args.output_path), Utilities.get_uniquie_id())

        CppAstParser.set_library_path(Utilities.get_abs_path(Constants.LIBCLANG_PATH))
        CppAstParser.set_standard_includes(os.path.join(Utilities.get_abs_path(Constants.LIBCLANG_PATH), "include"))

    @staticmethod
    def get_ast_as_json(compile_db_path):
        """
        Creates an instance of CppAstParser and gets all the tokens for various .cpp files as a python dict
        :param compile_db_path: path to compile_commands.json file
        :return: dict with key as path to .cpp and values as the tokens dict
        """
        cp = CppAstParser(compile_db_path, RosUpgrader.SRC_PATH_TO_UPGRADE)
        all_tokens = cp.get_ast_obj()
        RosUpgrader.src_file_list = cp.get_file_list()

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
            Utilities.write_to_file(RosUpgrader.get_output_filepath(file_path), ported_content)

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
            tree.write(RosUpgrader.get_output_filepath(file_path), encoding="utf-8", xml_declaration=True)

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
    def get_output_filepath(file_path):
        """
        Returns the full file path for the ported output file
        :param file_path: src path which was ported
        :return: str
        """
        common_prefix = os.path.commonprefix([Utilities.get_parent_dir(RosUpgrader.SRC_PATH_TO_UPGRADE), file_path])

        new_file_path = os.path.join(RosUpgrader.OUTPUT_PATH, file_path[len(common_prefix) + 1:])

        return new_file_path

    @staticmethod
    def convert_all_source_files():
        """
        Calls the porter on all source files to convert the ROS1 source to ROS2
        :return: None
        """
        RosUpgrader.consolidated_mapping = Utilities.get_consolidated_mapping()
        if RosUpgrader.consolidated_mapping is not None:
            diff_content = []
            for file_path in RosUpgrader.src_file_list:
                src_content = Utilities.read_from_file(file_path)

                cpp_porter = CPPSourceCodePorter(RosUpgrader.AST_DICT, file_path)

                if file_path in RosUpgrader.AST_LINE_BY_LINE:
                    new_src = cpp_porter.port(source=src_content,
                                              mapping=RosUpgrader.consolidated_mapping,
                                              ast=RosUpgrader.AST_LINE_BY_LINE[file_path])

                    Utilities.write_to_file(RosUpgrader.get_output_filepath(file_path), new_src)

                    diff_content.extend(Utilities.get_diff_content_of_files(src_content, new_src, file_path))
                else:
                    Utilities.write_to_file(RosUpgrader.get_output_filepath(file_path), src_content)

            Utilities.write_to_file(os.path.join(RosUpgrader.OUTPUT_PATH, Constants.DIFF_FILE_PATH),
                                    "\n".join(diff_content))

    @staticmethod
    def add_filled_mappings():
        """
        Adds the filled mapping from NEW_TOKENS_FILE to the `MAPPING_FILE_NAME` file
        :return: None
        """
        mapping_file_path = os.path.join(MappingConstants.MAPPING_FOLDER, RosUpgrader.MAPPING_FILE_NAME)
        new_mappings = Utilities.read_json_file(os.path.join(MappingConstants.MAPPING_FOLDER,
                                                             MappingConstants.NEW_TOKENS_FILE_NAME))

        if len(new_mappings.keys()) == 0:
            return

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
        """
        Returns true if new tokens has been added, which requires corresponding ROS2 mappings
        :param new_mappings: dict containing new tokens
        :return: boolean
        """
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
            prompt = str(input("Open mappings/new_tokens.json and fill the mappings. "
                               "Press 'Y' to continue or any other key to abort: "))
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
    return args.debug


def main():
    RosUpgrader.init()

    new_mappings = Utilities.get_new_tokens_template()

    already_added_mappings = {}

    for compile_json in RosUpgrader.COMPILE_JSON_LIST:
        RosUpgrader.add_new_mappings(compile_json, new_mappings, already_added_mappings)

    if is_debugging():
        Utilities.write_as_json("ast_dump_" + AstConstants.NON_UNIT_TEST + ".json",
                                RosUpgrader.AST_DICT[AstConstants.NON_UNIT_TEST])
        Utilities.write_as_json("ast_dump_" + AstConstants.UNIT_TEST + ".json",
                                RosUpgrader.AST_DICT[AstConstants.UNIT_TEST])
    RosUpgrader.start_upgrade(new_mappings)


if __name__ == '__main__':
    main()
