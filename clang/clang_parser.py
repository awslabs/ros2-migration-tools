#Copyright (c) 2017 Andre Santos
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

import json
import logging
import os
import re

import clang.cindex as clang

from Constants import AstConstants, ClangTokenKind, Constants, RosConstants
from utilities import Utilities

CK = clang.CursorKind


class CppAstParser(object):
    """
    Wrapper to use libclang python API.
    """
    lib_path = None  # path to folder containing libclang.so
    includes = None  # standard includes folder
    filter_out = None
    token_hash = set()

    READABLE_TOKEN_KIND = {
        ClangTokenKind.CALL_EXPR: AstConstants.CALL_EXPR,
        ClangTokenKind.CONVERSION_FUNCTION: AstConstants.CONVERSION_FUNCTION,
        ClangTokenKind.INCLUSION_DIRECTIVE: AstConstants.INCLUSION_DIRECTIVE,
        ClangTokenKind.MACRO_INSTANTIATION: AstConstants.MACRO_INSTANTIATION,
        ClangTokenKind.NAMESPACE_REF: AstConstants.NAMESPACE_REF,
        ClangTokenKind.PARM_DECL: AstConstants.PARM_DECL,
        ClangTokenKind.VAR_DECL: AstConstants.VAR_DECL,
        ClangTokenKind.MEMBER_REF_EXPR: AstConstants.MEMBER_REF_EXPR,
        ClangTokenKind.TYPE_REF: AstConstants.TYPE_REF,
        ClangTokenKind.DECL_REF_EXPR: AstConstants.DECL_REF_EXPR,
        ClangTokenKind.FUNCTION_DECL: AstConstants.FUNCTION_DECL,
        ClangTokenKind.CXX_METHOD: AstConstants.CXX_METHOD,
        ClangTokenKind.FIELD_DECL: AstConstants.FIELD_DECL,
        ClangTokenKind.CLASS_DECL: AstConstants.CLASS_DECL
    }

    @classmethod
    def set_library_path(cls, lib_path="/usr/lib/llvm-3.8/lib"):
        """
        Sets libclang.so folder path in python API
        :param lib_path: path to folder containing libclang.so
        :return: None
        """
        cls.filter_out = Utilities.read_json_file(Utilities.get_abs_path(os.path.join("clang", Constants.FILTER_OUT_FILE_PATH)))
        clang.Config.set_library_path(lib_path)
        cls.lib_path = lib_path

    @classmethod
    def set_standard_includes(cls, std_includes):
        """
        Sets standard include path
        :param std_includes:
        :return: None
        """
        cls.includes = std_includes

    @staticmethod
    def should_exclude_from_ast(path):
        """
        Returns True if tokens for this path are to be excluded from AST
        :param path: path to the src file
        :return: boolean
        """
        # exclude system includes
        if path.startswith(os.path.join(os.path.sep, "usr", "lib")) or \
                path.startswith(os.path.join(os.path.sep, "usr", "include")):
            return True

        for version in CppAstParser.filter_out[AstConstants.ROS_VERSIONS]:
            # exclude aws tokens
            if os.path.join(version, "include", "aws", "") in path:
                return True

            # exclude aws_common tokens
            if os.path.join(version, "include", "aws_common", "") in path:
                return True

        return False

    @staticmethod
    def _check_compilation_problems(translation_unit):
        """
        Logs the errors and warning in translation unit
        :param translation_unit: clang translation_unit
        :return:
        """
        if translation_unit.diagnostics:
            for diagnostic in translation_unit.diagnostics:
                if diagnostic.severity >= clang.Diagnostic.Error:
                    logging.warning(diagnostic.spelling)

    def _cursor_obj(self, cursor):
        """
        Returns dict containing token information
        :param cursor: clang cursor to get the informations from
        :return: dict
        """
        line = 0
        token_start_col = 0
        token_end_col = 0
        declaration_file_path = ""
        src_file_path = ""
        var_type = ""
        line_tokens = []
        try:
            if cursor.location.file:
                line = cursor.location.line
                token_start_col = cursor.extent.start.column
                token_end_col = cursor.extent.end.column
                src_file_path = cursor.location.file.name
        except AttributeError as e:
            pass
        name = repr(cursor.kind)[11:]
        if name in CppAstParser.READABLE_TOKEN_KIND:
            name = CppAstParser.READABLE_TOKEN_KIND[name]

        spell = cursor.spelling or AstConstants.NO_SPELLING

        if spell != AstConstants.NO_SPELLING:
            try:
                declaration_file_path = cursor.referenced.location.file.name
            except AttributeError as e:
                logging.warning(spell + ": declaration_file_name not present")

            try:
                var_type = cursor.type.spelling
            except AttributeError:
                logging.warning(spell + ": var_type not present")

            try:
                token_list = list(cursor.get_tokens())
                for i in range(0, len(token_list)):
                    line_tokens.append(token_list[i].spelling)

            except AttributeError as e:
                logging.warning("Couldn't get line_tokens")

        if name == AstConstants.FIELD_DECL:
            name = AstConstants.VAR_DECL

        if name == AstConstants.PARM_DECL or name == AstConstants.VAR_DECL:
            if "," not in var_type and "std::shared_ptr" not in var_type and var_type.startswith("std::"):  # not map and shared_ptr
                if "<" in var_type and ">" in var_type:
                    var_type = var_type[var_type.find("<") + 1: var_type.rfind(">")]

            #     var_type = re.sub(r"^std::shared_ptr<", "", var_type)
            #     var_type = var_type[:-1]
            #var_type = re.sub(r" *(\**)$", "", var_type)
            #var_type = re.sub(r" *(&*)$", "", var_type)
            var_type = re.sub(r"^const *", "", var_type)

        return {
            AstConstants.LINE: line,
            AstConstants.KIND: name,
            AstConstants.NAME: spell,
            AstConstants.DECL_FILEPATH: declaration_file_path,
            AstConstants.VAR_TYPE: var_type,
            AstConstants.LINE_TOKENS: line_tokens,
            AstConstants.SRC_FILE_PATH: src_file_path,
            AstConstants.TOKEN_START_COL: token_start_col,
            AstConstants.TOKEN_END_COL: token_end_col
        }

    @staticmethod
    def should_append_to_token_list(curr_obj):
        """
        Check if token dict is required in the tokens list
        :param curr_obj: token dict
        :return: boolean
        """
        if curr_obj[AstConstants.NAME] == AstConstants.NO_SPELLING:
            return False
        if CppAstParser.should_exclude_from_ast(curr_obj[AstConstants.SRC_FILE_PATH]):
            return False
        if CppAstParser.should_exclude_from_ast(curr_obj[AstConstants.DECL_FILEPATH]):
            return False

        if curr_obj[AstConstants.KIND] == AstConstants.INCLUSION_DIRECTIVE and \
                curr_obj[AstConstants.NAME] in CppAstParser.filter_out[AstConstants.INCLUSION_DIRECTIVE]:
            return False

        identifier = curr_obj[AstConstants.NAME]
        if curr_obj[AstConstants.KIND] == AstConstants.VAR_DECL or curr_obj[AstConstants.KIND] == AstConstants.PARM_DECL:
            identifier = curr_obj[AstConstants.VAR_TYPE]

        try:
            # removing "const", "static" etc from beginning
            for qualifier in CppAstParser.filter_out[AstConstants.TYPE_QUALIFIER]:
                pattern = qualifier + " "
                if identifier.startswith(pattern):
                    identifier = identifier[len(pattern):]

            for prim_type in CppAstParser.filter_out[AstConstants.VAR_DECL]:
                pattern = "^" + prim_type + "\\b"
                if re.search(pattern, identifier):
                    return False

            if curr_obj[AstConstants.KIND] == AstConstants.PARM_DECL or curr_obj[AstConstants.KIND] == AstConstants.VAR_DECL:
                if "std::shared_ptr" in identifier and \
                        re.search("\\b" + RosConstants.ROS1_NAMESPACE + "\\b", identifier):
                    return True

            for ns in CppAstParser.filter_out[AstConstants.NAMESPACE_REF]:
                if identifier.startswith(ns):
                    return False

        except Exception as e:
            logging.warning("should_append_to_token_list: couldn't remove const")
            return True

        return True

    def _add_token_to_ast(self, curr_obj, ast_obj):
        """
        Appends the `curr_obj` token dict to corresponding ast(either `UNIT_TEST` or `NON_UNIT_TEST`,
        if `should_append_to_token_list()` for it is True
        :param curr_obj: token attributes dict
        :param ast_obj: dict of token categories
        :return: None
        """
        # check for duplicates
        curr_obj_hash = hash(json.dumps(curr_obj, sort_keys=True))
        if curr_obj_hash in CppAstParser.token_hash:
            return
        else:
            CppAstParser.token_hash.add(curr_obj_hash)

        if curr_obj[AstConstants.SRC_FILE_PATH] != "":
            self.file_list.add(curr_obj[AstConstants.SRC_FILE_PATH])

        if CppAstParser.should_append_to_token_list(curr_obj):
            ast_of_category = ast_obj[AstConstants.NON_UNIT_TEST]
            if Utilities.is_unit_test_path(curr_obj[AstConstants.SRC_FILE_PATH]):
                ast_of_category = ast_obj[AstConstants.UNIT_TEST]

            if curr_obj[AstConstants.KIND] not in ast_of_category:
                ast_of_category[curr_obj[AstConstants.KIND]] = []

            ast_of_category[curr_obj[AstConstants.KIND]].append(curr_obj)

    def __init__(self, db_path, workspace=""):
        """
        Constructor for CPPAstParser
        :param db_path: Path to folder containing compile_commands.json
        :param workspace: path to folder containing the source code
        """
        self.workspace = os.path.abspath(workspace) if workspace else ""

        self._index = None

        self._db = clang.CompilationDatabase.fromDirectory(db_path)
        self._db.db_path = db_path
        self.file_list = set()

    def get_ast_obj(self, file_path=None):
        """
        Returns the dict of token kinds. Each kind will contain list of tokens
        :param file_path: src file path for which to get the AST
        :return: dict
        """
        if file_path is None:
            cmd = self._db.getAllCompileCommands() or ()
        else:
            cmd = self._db.getCompileCommands(os.path.abspath(file_path)) or ()

        ast_obj = {
            AstConstants.UNIT_TEST: {},
            AstConstants.NON_UNIT_TEST: {}
        }

        if not cmd:
            return None
        for c in cmd:
            if CppAstParser.should_exclude_from_ast(c.directory) or CppAstParser.should_exclude_from_ast(c.filename):
                continue

            with cwd(os.path.join(self._db.db_path, c.directory)):
                args = ["-I" + CppAstParser.includes] + list(c.arguments)[1:]
                if self._index is None:
                    self._index = clang.Index.create()
                unit = self._index.parse(path=None, args=args, options=clang.TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD)

                self._check_compilation_problems(unit)
                self._traverse_cursor(unit.cursor, ast_obj)
        return ast_obj

    def _traverse_cursor(self, top_cursor, ast_obj):
        """
        Traverses the cursor and adds the token to corresponding category in ast_obj
        :param top_cursor: starting cursor
        :param ast_obj: dict of tokens category
        :return: None
        """
        assert top_cursor.kind == CK.TRANSLATION_UNIT
        for cursor in top_cursor.get_children():
            if cursor.location.file and cursor.location.file.name.startswith(self.workspace):
                curr_obj = self._cursor_obj(cursor)
                self._add_token_to_ast(curr_obj, ast_obj)

                stack = list(cursor.get_children())
                while stack:
                    c = stack.pop()
                    curr_obj = self._cursor_obj(c)
                    self._add_token_to_ast(curr_obj, ast_obj)

                    stack.extend(c.get_children())

    def get_file_list(self):
        """
        Returns list containing full paths of files encountered in AST
        :return: list
        """
        return list(self.file_list)


###############################################################################
# Helpers
###############################################################################

class cwd:
    """Run a block of code from a specified working directory"""
    def __init__(self, path):
        self.dir = path

    def __enter__(self):
        self.old_dir = os.getcwd()
        os.chdir(self.dir)

    def __exit__(self, exc_type, exc_value, traceback):
        os.chdir(self.old_dir)
