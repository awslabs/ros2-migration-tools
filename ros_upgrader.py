from bonsai.cpp.clang_parser import CppAstParser
import os
import json


class RosUpgrader:
    compile_db_dir = None
    work_space = None
    includes = []

    @staticmethod
    def get_abs_path(rel_path):
        return os.path.abspath(rel_path)

    @staticmethod
    def init():
        try:
            config = open("config.json")
        except IOError as e:
            print(e)

        config_obj = json.load(config)

        CppAstParser.set_library_path(RosUpgrader.get_abs_path(config_obj['LIBCLANG_PATH']))
        print(RosUpgrader.get_abs_path(config_obj['COMPILE_DB_DIR']))
        CppAstParser.set_database(RosUpgrader.get_abs_path(config_obj['COMPILE_DB_DIR']))
        RosUpgrader.work_space = RosUpgrader.get_abs_path(config_obj['WORKSPACE_DIR'])
        RosUpgrader.includes = []
        RosUpgrader.compile_db_dir = RosUpgrader.get_abs_path(config_obj['COMPILE_DB_DIR'])
        for fpath in config_obj['USER_INCLUDES']:
            RosUpgrader.includes.append(RosUpgrader.get_abs_path(fpath))

    @staticmethod
    def get_cpp_file_list():
        file_list = []
        with open(os.path.join(RosUpgrader.compile_db_dir, "compile_commands.json")) as f:
            entries = json.load(f)
            for entry in entries:
                if ".cpp" in entry["file"]:
                    file_list.append(entry["file"])

        return file_list

    @staticmethod
    def get_tokens_as_json():
        cp = CppAstParser(RosUpgrader.work_space, RosUpgrader.includes)

        file_list = RosUpgrader.get_cpp_file_list()

        all_tokens = {}
        for file_path in file_list:
            all_tokens[file_path] = cp.get_ast_obj(file_path)

        return all_tokens

    def __init__(self):
        pass



