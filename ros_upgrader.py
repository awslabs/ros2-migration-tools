from bonsai.cpp.clang_parser import CppAstParser
from porting_tools.cmake_lists_porter import CMakeListsPorter
from porting_tools.package_xml_porter import PackageXMLPorter
from porting_tools.source_code_porter import SourceCodePorter
from Constants import AstConstants, Constants
from utils import Utilities
import xml.etree.ElementTree as etree
import os
import json
import fnmatch
import sys
import shutil

class RosUpgrader:
    """
    Convert ROS1 package to ROS2 package
    """
    TOKEN_TYPES = (
        "CALL_EXPR",
        "NAMESPACE_REF",
        "USING_DIRECTIVE"
    )

    # path to the folder containing "compile_commands.json" file, will be generated using cmake flag
    # "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    COMPILE_JSON_PATH = None

    # folder to upgrade
    SRC_PATH_TO_UPGRADE = None

    # path to all the include folders required by the package
    INCLUDES = []

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
        with open(RosUpgrader.COMPILE_JSON_PATH) as f:
            entries = json.load(f)
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
    def add_new_mappings():
        """
        Parses the file and adds the mappings to "mappings.json" if the mapping was missing
        :return: None
        """
        with open(Constants.MAPPING_FILE_NAME, 'r') as f:
            mappings = json.load(f)

        irrelevant_tokens = set(mappings[Constants.IRRELEVANT_TOKENS])

        tokens_dict = RosUpgrader.get_ast_as_json()

        #print(json.dumps(tokens_dict, indent=4))

        for file in tokens_dict:
            tokens = tokens_dict[file]
            #declared_func_list = RosUpgrader.get_declared_functions(tokens)
            #print(declared_func_list)
            for token_type in RosUpgrader.TOKEN_TYPES:
                added_set = set()
                if token_type in tokens:
                    for token in tokens[token_type]:
                        if token[AstConstants.NAME] != AstConstants.NO_SPELLING and \
                                token[AstConstants.NAME] not in irrelevant_tokens and \
                                token[AstConstants.NAME] not in added_set and \
                                Constants.ROS1_INCLUDE_PATH in token[Constants.DECL_FILEPATH]:

                            mappings[token_type][Constants.NEW_TOKENS_LIST].append({
                                Constants.ROS_1_NAME: token[AstConstants.NAME],
                                Constants.ROS_2_NAME: Constants.NEW_MAPPING_MSG
                            })
                            added_set.add(token[AstConstants.NAME])
                        else:
                            if token_type == AstConstants.USING_DIRECTIVE:
                                pass

        with open('mappings2.json', 'w') as f:
            json.dump(mappings, f, indent=4)

    @staticmethod
    def get_filled_mapping():
        """
        Reads the new mapping and return if valid
        :param matching:
        :return: True if matching is correctly filled otherwise false
        """
        try:
            with open(Constants.MAPPING_FILE_NAME) as f:
                return json.load(f)
        except BaseException :
            print("Couldn't read " + Constants.MAPPING_FILE_NAME)

        return None

    @staticmethod
    def convert_all_source_files():
        """
        Calls the porter on all source files to convert the ROS1 source to ROS2
        :return: None
        """
        mapping = RosUpgrader.get_filled_mapping()
        if mapping is not None:
            file_list = RosUpgrader.get_cpp_file_list()
            for file_path in file_list:
                with open(file_path, 'r') as f:
                    print(f.read())
                    new_src = SourceCodePorter.port(source=f.read(), mapping=mapping)
                    print(new_src)
                    with open(file_path, 'w') as f:
                        f.write(new_src)

    @staticmethod
    def add_filled_mappings():
        """
        Adds the filled mapping from NEW_TOKENS_LIST to the node type
        :return: None
        """
        with open(Constants.MAPPING_FILE_NAME) as f:
            mapping = json.load(f)
            for token_type in mapping:
                if token_type != Constants.IRRELEVANT_TOKENS:
                    new_token_list = mapping[token_type][Constants.NEW_TOKENS_LIST]
                    for new_token in new_token_list:
                        if new_token[Constants.ROS_2_NAME] != Constants.NEW_MAPPING_MSG:
                            updated_token = new_token
                            del updated_token[Constants.ROS_1_NAME]
                            mapping[token_type][new_token[Constants.ROS_1_NAME]] = updated_token
                        else:
                            mapping[Constants.IRRELEVANT_TOKENS].append(new_token[Constants.ROS_1_NAME])
                    mapping[token_type][Constants.NEW_TOKENS_LIST] = []

    @staticmethod
    def start_upgrade():
        """
        Calls functions required for the upgrade process
        :return: None
        """
        RosUpgrader.convert_all_cmake()
        RosUpgrader.convert_all_package_xml()
        RosUpgrader.add_new_mappings()

        prompt = str(raw_input("All mappings filled? Press 'Y' to continue or any other key to abort"))
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

    def __init__(self, compile_db_dir):
        pass


def test_code():
    RosUpgrader.SRC_PATH_TO_UPGRADE = "/Users/amanrja/Documents/ROS_2/upgraded_from_ros1/2019-06-28_20_19_03/src"  # sys.argv[1]
    shutil.rmtree(RosUpgrader.SRC_PATH_TO_UPGRADE)
    Utilities.copy_directory(RosUpgrader.SRC_PATH_TO_UPGRADE + "2", RosUpgrader.SRC_PATH_TO_UPGRADE)


def main():
    #uncomment this
    #RosUpgrader.SRC_PATH_TO_UPGRADE = sys.argv[1]

    #test code
    test_code()

    RosUpgrader.init()

    src_parent_dir = Utilities.get_parent_dir(RosUpgrader.SRC_PATH_TO_UPGRADE)

    compile_db_files = RosUpgrader.get_all_file_paths(src_parent_dir, Constants.COMPILE_COMMANDS_FILE)

    for compile_json in compile_db_files:
        RosUpgrader.COMPILE_JSON_PATH = compile_json
        RosUpgrader.set_compile_db(Utilities.get_parent_dir(compile_json))
        RosUpgrader.start_upgrade()

    # print(json.dumps(RosUpgrader.get_tokens_as_json(), indent=4))


if __name__ == '__main__':
    main()
