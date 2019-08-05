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

import logging
import os
import re

import clang.cindex as clang

from Constants import AstConstants
from utilities import Utilities

CK = clang.CursorKind


class CppAstParser(object):
    """
    Wrapper to use libclang python API.
    """
    lib_path = None  # path to folder containing libclang.so
    includes = None  # standard includes folder
    primitive_types_list = ["int", "char", "float", "double", "signed", "unsigned", "long", "short"]

    @classmethod
    def set_library_path(cls, lib_path="/usr/lib/llvm-3.8/lib"):
        """
        Sets libclang.so folder path in python API
        :param lib_path: path to folder containing libclang.so
        :return: None
        """
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
    def is_primitive_var_type_array(var_type):
        """
        Check if var_type is just an primitive type array
        :param var_type: token var type
        :return: boolean
        """
        for primitive_type in CppAstParser.primitive_types_list:
            pattern = "\\b" + primitive_type + "\\b" + " *\*?" + " *\\w+" + " *\["  # looking for pattern like char * var[
            if re.search(pattern, var_type) is not None:
                return True

        return False

    @staticmethod
    def should_exclude_from_ast(path):
        """
        Returns True if tokens for this path are to be excluded from AST
        :param path: path to the src file
        :return: boolean
        """

        # Assuming all unit test will follow path like `src/some_folder1/some_folder2/test/reader_test.cpp`
        # where 'test' occurs at least twice
        if path.count('test') > 1:
            return True

        # check if 'gtest' is part of path, then also exclude it
        if "gtest" in path:
            return True

        # exclude system includes
        if path.startswith(os.path.join(os.path.sep, "usr", "lib")) or \
                path.startswith(os.path.join(os.path.sep, "usr", "include")):
            return True

        # exclude aws tokens
        if os.path.join("kinetic", "include", "aws", "") in path:
            return True

        return False

    def __init__(self, workspace="", user_includes=None):
    # public:
        self.workspace      = os.path.abspath(workspace) if workspace else ""
        self.user_includes  = [] if user_includes is None else user_includes
    # private:
        self._index         = None
        self._db            = CppAstParser.database

    def get_ast_obj(self, file_path=None):
        return self._parse_from_db_as_obj(file_path)

    def _parse_from_db_as_obj(self, file_path):
        if file_path is None:
            cmd = self._db.getAllCompileCommands() or ()
        else:
            cmd = self._db.getCompileCommands(os.path.abspath(file_path)) or ()

        ast_obj = {}
        if not cmd:
            return None
        for c in cmd:
            if CppAstParser.exclude_from_ast(c.directory) or CppAstParser.exclude_from_ast(c.filename):
                continue

            with cwd(os.path.join(self._db.db_path, c.directory)):
                args = ["-I" + CppAstParser.includes] + list(c.arguments)[1:]
                if self._index is None:
                    self._index = clang.Index.create()
                unit = self._index.parse(path=None, args=args, options=clang.TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD)

                self._check_compilation_problems(unit)
                Utilities.merge_ast_dict(ast_obj, self._ast_obj(unit.cursor))
        return ast_obj

    def _ast_obj(self, top_cursor):
        assert top_cursor.kind == CK.TRANSLATION_UNIT
        objects = {}
        for cursor in top_cursor.get_children():
            if (cursor.location.file
                    and cursor.location.file.name.startswith(self.workspace)):
                curr_obj = self._cursor_obj(cursor)

                if curr_obj[AstConstants.KIND] not in objects:
                    objects[curr_obj[AstConstants.KIND]] = []

                if curr_obj[AstConstants.NAME] != AstConstants.NO_SPELLING and \
                        not CppAstParser.exclude_from_ast(curr_obj[AstConstants.SRC_FILE_PATH]) and \
                        not CppAstParser.exclude_from_ast(curr_obj[AstConstants.DECL_FILEPATH]):
                    objects[curr_obj[AstConstants.KIND]].append(curr_obj)

                stack = list(cursor.get_children())
                while stack:
                    c = stack.pop()
                    curr_obj = self._cursor_obj(c)
                    if curr_obj[AstConstants.KIND] not in objects:
                        objects[curr_obj[AstConstants.KIND]] = []

                    if curr_obj[AstConstants.NAME] != AstConstants.NO_SPELLING and \
                            not CppAstParser.exclude_from_ast(curr_obj[AstConstants.SRC_FILE_PATH]) and \
                            not CppAstParser.exclude_from_ast(curr_obj[AstConstants.DECL_FILEPATH]):

        # exclude aws_common tokens
        if os.path.join("kinetic", "include", "aws_common", "") in path:
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

    @staticmethod
    def _cursor_obj(cursor):
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

        if name == AstConstants.PARM_DECL:
            if "std::shared_ptr<" in var_type:
                var_type = re.sub(r"^std::shared_ptr<", "", var_type)
                var_type = var_type[:-1]
            var_type = re.sub(r" *(\**)$", "", var_type)
            var_type = re.sub(r" *(&*)$", "", var_type)
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

        if curr_obj[AstConstants.VAR_TYPE] == AstConstants.VAR_DECL and \
                CppAstParser.is_primitive_var_type_array(curr_obj[AstConstants.VAR_TYPE]):
            return False

        if curr_obj[AstConstants.VAR_TYPE] == AstConstants.PARM_DECL and \
                curr_obj[AstConstants.VAR_TYPE] in CppAstParser.primitive_types_list:
            return False

        return True

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

        ast_obj = {}
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
                Utilities.merge_ast_dict(ast_obj, self._traverse_cursor(unit.cursor))
        return ast_obj

    def _traverse_cursor(self, top_cursor):
        """
        Traverses the cursor and returns a dict containing token list for each kind
        :param top_cursor: starting cursor
        :return: dict
        """
        assert top_cursor.kind == CK.TRANSLATION_UNIT
        objects = {}
        for cursor in top_cursor.get_children():
            if (cursor.location.file
                    and cursor.location.file.name.startswith(self.workspace)):
                curr_obj = self._cursor_obj(cursor)

                if curr_obj[AstConstants.KIND] not in objects:
                    objects[curr_obj[AstConstants.KIND]] = []

                if CppAstParser.should_append_to_token_list(curr_obj):
                    objects[curr_obj[AstConstants.KIND]].append(curr_obj)

                stack = list(cursor.get_children())
                while stack:
                    c = stack.pop()
                    curr_obj = self._cursor_obj(c)
                    if curr_obj[AstConstants.KIND] not in objects:
                        objects[curr_obj[AstConstants.KIND]] = []

                    if CppAstParser.should_append_to_token_list(curr_obj):
                        objects[curr_obj[AstConstants.KIND]].append(curr_obj)

                    stack.extend(c.get_children())
        return objects


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
