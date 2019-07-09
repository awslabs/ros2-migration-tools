
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

###############################################################################
# Imports
###############################################################################

from collections import deque
from ctypes import ArgumentError
import os

import clang.cindex as clang

from ..parser import AnalysisData, MultipleDefinitionError
from .model import *
from Constants import AstConstants


###############################################################################
# Globals
###############################################################################

CK = clang.CursorKind


###############################################################################
# Notes to Self
###############################################################################

# This works with a queue of builders.
# Builders store cursors and other context specific information.
# The AST is built breadth-first, instead of depth-first.
# As the objects are built, they are added to their respective parents.

# After the program model is built, a traversal may be required to detect and
#   remove invalid or redundant constructs.


###############################################################################
# Language Entity Builders
###############################################################################

class CppEntityBuilder(object):
    def __init__(self, cursor, scope, parent, insert = None):
        self.scope  = scope
        self.parent = parent
        self.cursor = cursor
        self.insert_method = insert # how to link this entity to the parent
        self.file   = None
        self.line   = None
        self.column = None
        try:
            if cursor.location.file:
                self.file   = cursor.location.file.name
                self.line   = cursor.location.line
                self.column = cursor.location.column
        except ArgumentError as e:
            pass

    def build(self, data):
        """Build an object for the current cursor and
            corresponding builders for the cursor's children.
            Return None if an object cannot be built.
            Return (object, [builders]) otherwise.
        """
        return None

    # Let's add some methods here, just to avoid code duplication.

    def _build_variable(self, data):
        if self.cursor.kind == CK.VAR_DECL \
                or self.cursor.kind == CK.FIELD_DECL:
            id = self.cursor.get_usr()
            name = self.cursor.spelling
            result = self.cursor.type.spelling
            ctype = self.cursor.type.get_canonical().spelling
            cppobj = CppVariable(self.scope, self.parent, id, name, result,
                                 ctype=ctype)
            data.register(cppobj)
            builders = []
            children = list(self.cursor.get_children())
            if children and children[-1].kind != CK.TYPE_REF:
                b = CppExpressionBuilder(children[-1], self.scope, cppobj)
                builders.append(b)
            return (cppobj, builders)
        return None


    def _lookup_parent(self, cls):
        cppobj = self.parent
        while not cppobj is None and not isinstance(cppobj, cls):
            cppobj = cppobj.parent
        return cppobj



class CppExpressionBuilder(CppEntityBuilder):
    def __init__(self, cursor, scope, parent, insert = None):
        CppEntityBuilder.__init__(self, cursor, scope, parent, insert = insert)
        self.name   = cursor.spelling
        self.result = cursor.type.spelling or "[type]"
        self.parenthesis = False

    def build(self, data):
        result = self._pre_process_strings()
        if result is None:
            result = self._build_literal()
        if result is None:
            result = self._build_reference(data)
        if result is None:
            result = self._build_operator()
        if result is None:
            result = self._build_function_call(data)
        if result is None:
            result = self._build_default_argument()
        if result is None:
            result = self._build_other(data)
        if result is None:
            result = self._build_unexposed(data)
        return result


    def _pre_process_strings(self):
        if self.cursor.kind == CK.CALL_EXPR and self.name == "basic_string":
            cursor = next(self.cursor.get_children(), None)
            if not cursor:
                return ("", ())
            if cursor.kind == CK.UNEXPOSED_EXPR:
                cursor = next(cursor.get_children(), None)
                if cursor and (cursor.kind == CK.STRING_LITERAL
                               or cursor.kind == CK.DECL_REF_EXPR):
                    self.cursor = cursor
                    self.name = cursor.spelling
        return None

    def _build_literal(self):
        token = next(self.cursor.get_tokens(), None)
        if self.cursor.kind == CK.INTEGER_LITERAL:
            if token:
                token = token.spelling
                while token.endswith(("U", "u", "L", "l")):
                    token = token[:-1]
                return (int(token, 0), ())
            return (SomeCpp.INTEGER, ())
        if self.cursor.kind == CK.FLOATING_LITERAL:
            if token:
                if token.spelling[-1].isalpha():
                    return (float(token.spelling[:-1]), ())
                return (float(token.spelling), ())
            return (SomeCpp.FLOATING, ())
        if self.cursor.kind == CK.CHARACTER_LITERAL:
            return (token.spelling, ()) if token else (SomeCpp.CHARACTER, ())
        if self.cursor.kind == CK.CXX_BOOL_LITERAL_EXPR:
            return (token.spelling == "true", ()) if token \
                                                  else (SomeCpp.BOOL, ())
        if self.cursor.kind == CK.STRING_LITERAL:
            if self.name.startswith('"'):
                self.name = self.name[1:-1]
            return (self.name, ())
        return None

    def _build_reference(self, data):
        if self.cursor.kind == CK.DECL_REF_EXPR \
                or self.cursor.kind == CK.MEMBER_REF \
                or self.cursor.kind == CK.MEMBER_REF_EXPR:
            ctype = self.cursor.type.get_canonical().spelling
            cppobj = CppReference(self.scope, self.parent,
                                  self.name, self.result, ctype=ctype)
            cppobj.parenthesis = self.parenthesis
            cppobj.file = self.file
            cppobj.line = self.line
            cppobj.column = self.column
            ref = self.cursor.get_definition()
            if ref:
                data.reference(ref.get_usr(), cppobj)
            if self.cursor.kind == CK.MEMBER_REF_EXPR:
                cursor = next(self.cursor.get_children(), None)
                if cursor:
                    builder = CppExpressionBuilder(cursor, self.scope, cppobj,
                                                   insert = cppobj._set_field)
                    return (cppobj, (builder,))
                else:
                    ctype = self.cursor.type.get_canonical().spelling
                    cppobj.field_of = CppReference(self.scope, cppobj,
                                                    "this", "[type]", ctype=ctype)
            return (cppobj, ())
        if self.cursor.kind == CK.CXX_THIS_EXPR:
            ctype = self.cursor.type.get_canonical().spelling
            cppobj = CppReference(self.scope, self.parent, "this", self.result,
                                  ctype=ctype)
            cppobj.parenthesis = self.parenthesis
            return (cppobj, ())
        return None

    def _build_operator(self):
        # TODO conditional operator
        name = None
        if self.cursor.kind == CK.UNARY_OPERATOR:
            name = self._parse_unary_operator()
        elif self.cursor.kind == CK.BINARY_OPERATOR \
                or self.cursor.kind == CK.COMPOUND_ASSIGNMENT_OPERATOR:
            name = self._parse_binary_operator()
        if not name is None:
            ctype = self.cursor.type.get_canonical().spelling
            cppobj = CppOperator(self.scope, self.parent, name, self.result,
                                 ctype=ctype)
            cppobj.parenthesis = self.parenthesis
            cppobj.file = self.file
            cppobj.line = self.line
            cppobj.column = self.column
            builders = [CppExpressionBuilder(c, self.scope, cppobj) \
                        for c in self.cursor.get_children()]
            return (cppobj, builders)
        return None

    def _build_function_call(self, data):
        if self.cursor.kind == CK.CXX_NEW_EXPR:
            self.cursor = list(self.cursor.get_children())[-1]
            self.name = self.cursor.spelling
        # NOTE: this is not totally correct, needs revision
        if self.cursor.kind == CK.CALL_EXPR:
            if self.name:
                ctype = self.cursor.type.get_canonical().spelling
                cppobj = CppFunctionCall(self.scope, self.parent,
                                         self.name, self.result, ctype=ctype)
                cppobj.file = self.file
                cppobj.line = self.line
                cppobj.column = self.column
                cppobj.parenthesis = self.parenthesis
    # ----- this is still tentative -------------------------------------------
                tokens = [t.spelling for t in self.cursor.get_tokens()]
                try:
                    cppobj.full_name = "".join(tokens[:tokens.index("(")])
                except ValueError as e:
                    if cppobj.is_constructor:
                        cppobj.full_name = "".join((cppobj.canonical_type,
                                                    "::", cppobj.name))
                cppobj.template = self._parse_templates(cppobj.name,
                                                        "".join(tokens))
                # TODO not sure whether parsing only the type is enough
                if not cppobj.template:
                    cppobj.template = self._parse_templates(
                        cppobj.name, cppobj.canonical_type)
    # -------------------------------------------------------------------------
                ref = self.cursor.get_definition() or self.cursor.referenced
                if ref:
                    data.reference(ref.get_usr(), cppobj)
                builders = []
                args = list(self.cursor.get_arguments())
                for cursor in args:
                    builders.append(CppExpressionBuilder(cursor,
                                    self.scope, cppobj))
                child_is_arg = not args and cppobj.is_constructor
                for cursor in self.cursor.get_children():
                    if (cursor.kind == CK.MEMBER_REF_EXPR
                            and cursor.spelling == self.name):
                        children = list(cursor.get_children())
                        if not children:
                            continue
                        builders.append(CppExpressionBuilder(children[0],
                                        self.scope, cppobj,
                                        insert = cppobj._set_method))
                    elif child_is_arg:
                        builders.append(CppExpressionBuilder(cursor,
                                        self.scope, cppobj))
                return (cppobj, builders)
            else:
                result = None
                cursor = next(self.cursor.get_children(), None)
                if not cursor is None:
                    original    = self.cursor
                    self.cursor = cursor
                    self.name   = cursor.spelling
                    result      = self.build(data)
                    self.cursor = original
                elif isinstance(self.parent, CppVariable):
                    self.name   = self.result.split(":")[-1]
                    result      = self._build_function_call(data)
                return result
        elif self.cursor.kind == CK.CXX_DELETE_EXPR:
            ctype = self.cursor.type.get_canonical().spelling
            cppobj = CppFunctionCall(self.scope, self.parent,
                                     "delete", self.result, ctype=ctype)
            cppobj.file = self.file
            cppobj.line = self.line
            cppobj.column = self.column
            cppobj.parenthesis = self.parenthesis
            ref = next(self.cursor.get_children())
            builder = CppExpressionBuilder(ref, self.scope, cppobj)
            return (cppobj, (builder,))
        return None

    def _build_default_argument(self):
        if isinstance(self.parent, CppFunctionCall) \
                and self.cursor.kind == CK.UNEXPOSED_EXPR \
                and not next(self.cursor.get_children(), None):
            ctype = self.cursor.type.get_canonical().spelling
            cppobj = CppDefaultArgument(self.scope, self.parent, self.result,
                                        ctype=ctype)
            cppobj.parenthesis = self.parenthesis
            cppobj.file = self.file
            cppobj.line = self.line
            cppobj.column = self.column
            return (cppobj, ())
        return None

    def _build_other(self, data):
        # This is skip behaviour: parse child instead.
        if self.cursor.kind == CK.PAREN_EXPR \
                or self.cursor.kind == CK.CSTYLE_CAST_EXPR:
            original = self.cursor
            children = list(self.cursor.get_children())
            self.parenthesis = original.kind == CK.PAREN_EXPR
            result = None
            if children:
                self.cursor = children[-1]
                self.name = self.cursor.spelling
                result = self.build(data)
            self.cursor = original
            return result
        return None

    def _build_unexposed(self, data):
        if self.cursor.kind == CK.CXX_FUNCTIONAL_CAST_EXPR:
            children = list(self.cursor.get_children())
            if children and children[-1].kind == CK.UNEXPOSED_EXPR:
                self.cursor = children[-1]
        if self.cursor.kind == CK.UNEXPOSED_EXPR:
            cursor = next(self.cursor.get_children(), None)
            if cursor:
                self.cursor = cursor
                self.name   = cursor.spelling
                return self.build(data)
        return None
    

    def _parse_unary_operator(self):
        tokens = list(self.cursor.get_tokens())
        if tokens:
            token = tokens[0].spelling
            if token in CppOperator._UNARY_TOKENS:
                return token
            # The last token seems to be what ends the expression, e.g. ';'
            token = tokens[-2].spelling
            if token in CppOperator._UNARY_TOKENS:
                if token == "++" or token == "--":
                    return "_" + token
                return token
        return "[op]"

    def _parse_binary_operator(self):
        # There are no alpha operators
        # I think "->" and "->*" might have their own CursorKind
        # All operators seem to be infix; get the last token of the first child
        child = next(self.cursor.get_children(), None)
        if child:
            tokens = list(child.get_tokens())
            if tokens:
                token = tokens[-1].spelling
                if token in CppOperator._BINARY_TOKENS:
                    return token
        return "[op]"

    def _parse_templates(self, name, text):
        templates = []
        start = text.find("<")
        if (start >= 0 and not "<" in name and not ">" in name
                       and text[:start].endswith(name)):
            matches = 1
            i = start + 1
            while matches > 0 and i < len(text):
                if text[i] == "<":
                    matches += 1
                elif text[i] == ">":
                    matches -= 1
                elif text[i] == "," and matches == 1:
                    templates.append(text[start+1:i])
                    start = i
                i += 1
            templates.append(text[start+1:i-1])
        return tuple(templates)


class CppStatementBuilder(CppEntityBuilder):
    def __init__(self, cursor, scope, parent, insert = None):
        CppEntityBuilder.__init__(self, cursor, scope, parent, insert = insert)

    def build(self, data):
        result = self._build_declarations(data)
        result = result or self._build_expression(data)
        result = result or self._build_control_flow()
        result = result or self._build_jump_statement()
        result = result or self._build_block()
        result = result or self._build_try_block(data)
        result = result or self._build_unexposed(data)
        result = result or self._build_label_statement(data)
        return result


    def _build_expression(self, data):
        builder = CppExpressionBuilder(self.cursor, self.scope, self.parent)
        result = builder.build(data)
        if result:
            expression = result[0]
            cppobj = CppExpressionStatement(self.scope, self.parent,
                                            expression = expression)
            cppobj.file = self.file
            cppobj.line = self.line
            cppobj.column = self.column
            if isinstance(expression, CppExpression):
                expression.parent = cppobj
            result = (cppobj, result[1])
        return result

    def _build_declarations(self, data):
        if self.cursor.kind == CK.DECL_STMT:
            cppobj = CppDeclaration(self.scope, self.parent)
            cppobj.file = self.file
            cppobj.line = self.line
            cppobj.column = self.column
            original = self.cursor
            self.parent = cppobj
            builders = []
            for cursor in original.get_children():
                self.cursor = cursor
                result = self._build_variable(data)
                if result:
                    cppobj._add(result[0])
                    builders.extend(result[1])
            self.cursor = original
            self.parent = cppobj.parent
            return (cppobj, builders)
        return None

    def _build_control_flow(self):
        if not next(self.cursor.get_tokens(), None):
            # This is to try to avoid ROS_INFO and similar things.
            return None
        if self.cursor.kind == CK.WHILE_STMT:
            return self._build_while_statement()
        if self.cursor.kind == CK.FOR_STMT:
            return self._build_for_statement()
        if self.cursor.kind == CK.DO_STMT:
            return self._build_do_statement()
        if self.cursor.kind == CK.IF_STMT:
            return self._build_if_statement()
        if self.cursor.kind == CK.SWITCH_STMT:
            return self._build_switch_statement()
        return None

    def _build_jump_statement(self):
        if self.cursor.kind == CK.RETURN_STMT:
            cppobj = CppJumpStatement(self.scope, self.parent, "return")
            cppobj.file = self.file
            cppobj.line = self.line
            cppobj.column = self.column
            expression  = next(self.cursor.get_children(), None)
            builders = [CppExpressionBuilder(expression, self.scope, cppobj)] \
                        if expression else ()
            return (cppobj, builders)
        if self.cursor.kind == CK.BREAK_STMT:
            cppobj = CppJumpStatement(self.scope, self.parent, "break")
            cppobj.file = self.file
            cppobj.line = self.line
            cppobj.column = self.column
            return (cppobj, ())
        if self.cursor.kind == CK.CONTINUE_STMT:
            cppobj = CppJumpStatement(self.scope, self.parent, "continue")
            cppobj.file = self.file
            cppobj.line = self.line
            cppobj.column = self.column
            return (cppobj, ())
        return None

    def _build_block(self):
        if self.cursor.kind == CK.NULL_STMT:
            return None
        if self.cursor.kind == CK.COMPOUND_STMT:
            cppobj = CppBlock(self.scope, self.parent, explicit = True)
            builders = [CppStatementBuilder(c, cppobj, cppobj) \
                        for c in self.cursor.get_children()]
            return (cppobj, builders)
        return None

    def _build_try_block(self, data):
        if self.cursor.kind == CK.CXX_CATCH_STMT:
            return self._build_catch_block(data)
        if self.cursor.kind != CK.CXX_TRY_STMT:
            return None
        cppobj = CppTryBlock(self.scope, self.parent)
        cppobj.file = self.file
        cppobj.line = self.line
        cppobj.column = self.column
        builders = []
        children = list(self.cursor.get_children())
        assert len(children) >= 1
        builders.append(CppStatementBuilder(children[0], self.scope, cppobj,
                                            insert = cppobj._set_body))
        for i in xrange(1, len(children)):
            assert children[i].kind == CK.CXX_CATCH_STMT
            builders.append(CppStatementBuilder(children[i], self.scope, cppobj,
                                                insert = cppobj._add_catch))
        return (cppobj, builders)

    def _build_catch_block(self, data):
        cppobj = CppCatchBlock(self.scope, self.parent)
        cppobj.file = self.file
        cppobj.line = self.line
        cppobj.column = self.column
        builders = []
        children = list(self.cursor.get_children())
        assert len(children) == 1 or len(children) == 2
        if len(children) > 1:
            decl = CppDeclaration(cppobj, cppobj)
            original = self.cursor
            self.scope = cppobj
            self.parent = decl
            self.cursor = children[0]
            result = self._build_variable(data)
            if result:
                decl._add(result[0])
                decl.file = self.file
                decl.line = result[0].line
                decl.column = result[0].column
                cppobj._set_declarations(decl)
                builders.extend(result[1])
            self.cursor = original
            self.parent = cppobj.parent
            self.scope = cppobj.scope
        builders.append(CppStatementBuilder(children[-1], self.scope, cppobj,
                                            insert = cppobj._set_body))
        return (cppobj, builders)


    def _build_while_statement(self):
        cppobj = CppLoop(self.scope, self.parent, "while")
        cppobj.file = self.file
        cppobj.line = self.line
        cppobj.column = self.column
        builders = []
        children = list(self.cursor.get_children())
        assert len(children) >= 2
        builders.append(CppExpressionBuilder(children[0], self.scope, cppobj,
                                             insert = cppobj._set_condition))
        builders.append(CppStatementBuilder(children[1], self.scope, cppobj,
                                            insert = cppobj._set_body))
        return (cppobj, builders)

    def _build_for_statement(self):
        """NOTE: this is not a complete implementation of for loop parsing.
            Turns out for loops allow a number of wacky things going on,
            such as declaring a variable in place of a condition.
            These more peculiar cases are not covered. See
            http://en.cppreference.com/w/cpp/language/for
        """
        cppobj = CppLoop(self.scope, self.parent, "for")
        cppobj.file = self.file
        cppobj.line = self.line
        cppobj.column = self.column
        builders = []
        children = list(self.cursor.get_children())
        assert len(children) >= 1
        # body always comes last
        builders.append(CppStatementBuilder(children[-1], self.scope, cppobj,
                                            insert = cppobj._set_body))
        if len(children) == 1:
    # ----- just body -------------------------------------
            cppobj.condition = True
        elif len(children) == 2:
    # ----- condition + body ------------------------------
            builders.append(CppExpressionBuilder(children[0], self.scope,
                            cppobj, insert = cppobj._set_condition))
        elif len(children) >= 4:
    # ----- var + condition + increment + body ------------
            builders.append(CppStatementBuilder(children[0], cppobj,
                            cppobj, insert = cppobj._set_declarations))
            builders.append(CppExpressionBuilder(children[1], self.scope,
                            cppobj, insert = cppobj._set_condition))
            builders.append(CppStatementBuilder(children[2], cppobj,
                            cppobj, insert = cppobj._set_increment))
        elif children[0].kind == clang.CursorKind.DECL_STMT:
    # ----- var + condition + body ------------------------
            builders.append(CppStatementBuilder(children[0], cppobj,
                            cppobj, insert = cppobj._set_declarations))
            builders.append(CppExpressionBuilder(children[1], self.scope,
                            cppobj, insert = cppobj._set_condition))
        else:
    # ----- condition + increment + body ------------------
            builders.append(CppExpressionBuilder(children[0], self.scope,
                            cppobj, insert = cppobj._set_condition))
            builders.append(CppStatementBuilder(children[1], cppobj,
                            cppobj, insert = cppobj._set_increment))
        return (cppobj, builders)

    def _build_do_statement(self):
        cppobj = CppLoop(self.scope, self.parent, "do")
        cppobj.file = self.file
        cppobj.line = self.line
        cppobj.column = self.column
        builders = []
        children = list(self.cursor.get_children())
        assert len(children) >= 2
        builders.append(CppStatementBuilder(children[0], self.scope, cppobj,
                                            insert = cppobj._set_body))
        builders.append(CppExpressionBuilder(children[1], self.scope, cppobj,
                                             insert = cppobj._set_condition))
        return (cppobj, builders)

    def _build_if_statement(self):
        cppobj = CppConditional(self.scope, self.parent)
        cppobj.file = self.file
        cppobj.line = self.line
        cppobj.column = self.column
        builders = []
        children = list(self.cursor.get_children())
        assert len(children) >= 2
        builders.append(CppExpressionBuilder(children[0], self.scope, cppobj,
                                             insert = cppobj._set_condition))
        builders.append(CppStatementBuilder(children[1], self.scope, cppobj,
                                            insert = cppobj._set_body))
        if len(children) >= 3:
            # this is the "else" branch
            builders.append(CppStatementBuilder(children[2], self.scope, cppobj,
                            insert = cppobj._add_default_branch))
        return (cppobj, builders)

    def _build_switch_statement(self):
        """NOTE:
            This is not a complete implementation of switch statement parsing.
            The switch statement is probably one of the ugliest and less
            restrictive things in the language, and I am not going to support
            every possible case, especially not the pathological ones.
            See http://en.cppreference.com/w/cpp/language/switch
        """
        """The idea is to simply parse a body for the switch.
            Whenever a case is found, look up its switch parent and notify it.
            Return the result of parsing the statement within the case.
            Any statement body should assign a statement index to its children.
            This way, a jump can be represented as a parent plus index.
        """
        cppobj = CppSwitch(self.scope, self.parent)
        cppobj.file = self.file
        cppobj.line = self.line
        cppobj.column = self.column
        builders = []
        children = list(self.cursor.get_children())
        assert len(children) >= 2
        builders.append(CppExpressionBuilder(children[0], self.scope, cppobj,
                                             insert = cppobj._set_condition))
        builders.append(CppStatementBuilder(children[1], self.scope, cppobj,
                                            insert = cppobj._set_body))
        return (cppobj, builders)

    def _build_label_statement(self, data):
        original = self.cursor
        if self.cursor.kind == CK.CASE_STMT:
            switch = self._lookup_parent(CppSwitch)
            children = list(self.cursor.get_children())
            value = CppExpressionBuilder(children[0], self.scope, self.parent)
            value = value.build(data)
            self.cursor = children[1]
            result = self.build(data)
            if result:
                switch._add_branch(value, result[0])
            self.cursor = original
            return result
        if self.cursor.kind == CK.DEFAULT_STMT:
            switch = self._lookup_parent(CppSwitch)
            self.cursor = next(self.cursor.get_children())
            result = self.build(data)
            if result:
                switch._add_default_branch(result[0])
            self.cursor = original
            return result
        # TODO if self.cursor.kind == CK.LABEL_STMT:
        return None

    def _build_unexposed(self, data):
        if self.cursor.kind == CK.UNEXPOSED_STMT:
            cursor = next(self.cursor.get_children(), None)
            if cursor:
                self.cursor = cursor
                return self.build(data)
        return None



class CppTopLevelBuilder(CppEntityBuilder):
    def __init__(self, cursor, scope, parent, insert = None, workspace = ""):
        CppEntityBuilder.__init__(self, cursor, scope, parent, insert = insert)
        self.name = cursor.spelling
        self.workspace = workspace

    def build(self, data):
        result = self._build_variable(data)
        result = result or self._build_function(data)
        result = result or self._build_class(data)
        result = result or self._build_namespace()
        return result


    _FUNCTIONS = (CK.FUNCTION_DECL, CK.FUNCTION_TEMPLATE, CK.CXX_METHOD,
                  CK.CONSTRUCTOR, CK.DESTRUCTOR)

    def _build_function(self, data):
        # NOTE: function and method declarations only have children
        #       for their parameters. Only definitions have more.
        #       Skip declarations?
        if self.cursor.kind in CppTopLevelBuilder._FUNCTIONS:
            id = self.cursor.get_usr()
            result = self.cursor.result_type.spelling
            ctype = self.cursor.type.get_canonical().spelling
            cppobj = CppFunction(self.scope, self.parent, id,
                                 self.name, result, ctype=ctype)
            builders = []
            declaration = True
            children = self.cursor.get_children()
            cursor = next(children, None)
            while cursor:
                if cursor.kind == CK.PARM_DECL:
                    id = cursor.get_usr()
                    name = cursor.spelling or cursor.displayname
                    result = cursor.type.spelling or "[type]"
                    ctype = cursor.type.get_canonical().spelling or "[type]"
                    var = CppVariable(cppobj, cppobj, id, name, result,
                                      ctype=ctype)
                    data.register(var)
                    cppobj.parameters.append(var)
                elif cursor.kind == CK.TEMPLATE_TYPE_PARAMETER:
                    cppobj.template_parameters += 1
                elif cursor.kind == CK.MEMBER_REF:
                    # This is for constructors, we need the sibling
                    declaration = False
                    result  = cursor.type.spelling or "[type]"
                    ctype   = cursor.type.get_canonical().spelling or "[type]"
                    op      = CppOperator(cppobj, cppobj, "=", result,
                                          ctype=ctype)
                    member  = CppExpressionBuilder(cursor, cppobj, op)
                    cursor  = next(children)
                    value   = CppExpressionBuilder(cursor, cppobj, op)
                    stmt    = CppExpressionStatement(cppobj, cppobj, op)
                    op.parent = stmt
                    cppobj._add(stmt)
                    builders.append(member)
                    builders.append(value)
                elif cursor.kind == CK.COMPOUND_STMT:
                    declaration = False
                    for c in cursor.get_children():
                        builders.append(CppStatementBuilder(c, cppobj, cppobj))
                cursor = next(children, None)
            cppobj._definition = cppobj if not declaration else None
            try:
                data.register(cppobj, declaration=declaration)
            except MultipleDefinitionError as e:
                return None # TODO warning
            return (cppobj, builders)
        return None

    _CLASSES = (CK.CLASS_DECL, CK.STRUCT_DECL)

    def _build_class(self, data):
        if self.cursor.kind in CppTopLevelBuilder._CLASSES:
            id = self.cursor.get_usr()
            cppobj = CppClass(self.scope, self.parent, id, self.name)
            builders = []
            declaration = True
            for cursor in self.cursor.get_children():
                if cursor.kind == CK.CXX_BASE_SPECIFIER:
                    cppobj.superclasses.append(cursor.spelling)
                else:
                    declaration = False
                    builders.append(CppTopLevelBuilder(cursor, cppobj, cppobj))
            cppobj._definition = cppobj if not declaration else None
            try:
                data.register(cppobj, declaration=declaration)
            except MultipleDefinitionError as e:
                return None # TODO warning
            return (cppobj, builders)
        return None

    def _build_namespace(self):
        if self.cursor.kind == CK.NAMESPACE:
            cppobj = CppNamespace(self.scope, self.parent, self.name)
            builders = [CppTopLevelBuilder(c, cppobj, cppobj) \
                        for c in self.cursor.get_children()]
            return (cppobj, builders)
        return None



###############################################################################
# AST Parsing
###############################################################################

class CppAstParser(object):
    lib_path = None
    lib_file = None
    includes = "/usr/lib/llvm-3.8/lib/clang/3.8.0/include"
    database = None

    # system required / user optional
    @staticmethod
    def set_library_path(lib_path = "/usr/lib/llvm-3.8/lib"):
        clang.Config.set_library_path(lib_path)
        CppAstParser.lib_path = lib_path

    @staticmethod
    def set_library_file(lib_file = "/usr/lib/llvm-3.8/lib/libclang.so"):
        clang.Config.set_library_file(lib_file)
        CppAstParser.lib_file = lib_file

    # optional
    @staticmethod
    def set_database(db_path):
        if not CppAstParser.lib_path:
            CppAstParser.set_library_path()
        CppAstParser.database = clang.CompilationDatabase.fromDirectory(db_path)
        CppAstParser.database.db_path = db_path

    # optional
    @staticmethod
    def set_standard_includes(std_includes):
        CppAstParser.includes = std_includes

    def __init__(self, workspace = "", user_includes = None):
    # public:
        self.workspace      = os.path.abspath(workspace) if workspace else ""
        self.global_scope   = CppGlobalScope()
        self.data           = AnalysisData()
        self.user_includes  = [] if user_includes is None else user_includes
    # private:
        self._index         = None
        self._db            = CppAstParser.database

    def parse(self, file_path):
        file_path = os.path.abspath(file_path)
        if self._db is None:
            return self._parse_without_db(file_path)
        return self._parse_from_db(file_path)

    def get_ast_str(self, file_path):
        file_path = os.path.abspath(file_path)
        # if self._db is None:
        #     return self._parse_without_db(file_path, just_ast = True)
        return self._parse_from_db(file_path, just_ast = True)

    def get_ast_obj(self, file_path):
        file_path = os.path.abspath(file_path)
        return self._parse_from_db_as_obj(file_path)

    def _parse_from_db_as_obj(self, file_path):
    # ----- command retrieval -------------------------------------------------
        cmd = self._db.getCompileCommands(file_path) or ()
        if not cmd:
            return None
        for c in cmd:
            with cwd(os.path.join(self._db.db_path, c.directory)):
                args = ["-I" + CppAstParser.includes] + list(c.arguments)[1:]
                if self._index is None:
                    self._index = clang.Index.create()
                unit = self._index.parse(path=None, args=args, options=clang.TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD)
                self._check_compilation_problems(unit)
                return self._ast_obj(unit.cursor)

    def _parse_from_db(self, file_path, just_ast = False):
    # ----- command retrieval -------------------------------------------------
        cmd = self._db.getCompileCommands(file_path) or ()
        if not cmd:
            return None
        for c in cmd:
            with cwd(os.path.join(self._db.db_path, c.directory)):
                args = ["-I" + CppAstParser.includes] + list(c.arguments)[1:]
                if self._index is None:
                    self._index = clang.Index.create()
    # ----- parsing and AST analysis ------------------------------------------
                unit = self._index.parse(None, args)
                self._check_compilation_problems(unit)
                if just_ast:
                    return self._ast_str(unit.cursor)
                self._ast_analysis(unit.cursor)
        self.global_scope._afterpass()
        return self.global_scope

    def _parse_without_db(self, file_path, just_ast = False):
    # ----- command retrieval -------------------------------------------------
        with cwd(os.path.dirname(file_path)):
            args = ["-I" + CppAstParser.includes]
            for include_dir in self.user_includes:
                args.append("-I" + include_dir)
            args.append(file_path)
            if self._index is None:
                self._index = clang.Index.create()
    # ----- parsing and AST analysis ------------------------------------------
            unit = self._index.parse(None, args)
            self._check_compilation_problems(unit)
            if just_ast:
                return self._ast_str(unit.cursor)
            self._ast_analysis(unit.cursor)
        self.global_scope._afterpass()
        return self.global_scope

    def _ast_analysis(self, top_cursor):
        assert top_cursor.kind == CK.TRANSLATION_UNIT
        cppobj = self.global_scope
        builders = [CppTopLevelBuilder(c, cppobj, cppobj) \
                    for c in top_cursor.get_children() \
                    if c.location.file \
                        and c.location.file.name.startswith(self.workspace)]
        queue = deque(builders)
        while queue:
            builder = queue.popleft()
            result = builder.build(self.data)
            if result:
                cppobj, builders = result
                if builder.insert_method:
                    builder.insert_method(cppobj)
                else:
                    builder.parent._add(cppobj)
                queue.extend(builders)

    def _ast_obj(self, top_cursor):
        assert top_cursor.kind == CK.TRANSLATION_UNIT
        objects = {}
        for cursor in top_cursor.get_children():
            if (cursor.location.file
                    and cursor.location.file.name.startswith(self.workspace)):
                curr_obj = self._cursor_obj(cursor)
                if curr_obj["kind"] not in objects:
                    objects[curr_obj["kind"]] = []
                objects[curr_obj["kind"]].append(curr_obj)
                stack = list(cursor.get_children())
                while stack:
                    c = stack.pop()
                    curr_obj = self._cursor_obj(c)
                    if curr_obj["kind"] not in objects:
                        objects[curr_obj["kind"]] = []
                    objects[curr_obj["kind"]].append(curr_obj)
                    stack.extend(c.get_children())
        return objects

    def _ast_str(self, top_cursor):
        assert top_cursor.kind == CK.TRANSLATION_UNIT
        lines = []
        for cursor in top_cursor.get_children():
            if (cursor.location.file
                    and cursor.location.file.name.startswith(self.workspace)):
                lines.append(self._cursor_str(cursor, 0))
                indent = 0
                stack = list(cursor.get_children())
                stack.append(1)
                while stack:
                    c = stack.pop()
                    if isinstance(c, int):
                        indent += c
                    else:
                        lines.append(self._cursor_str(c, indent))
                        stack.append(-1)
                        stack.extend(c.get_children())
                        stack.append(1)
        return "\n".join(lines)

    def _check_compilation_problems(self, translation_unit):
        if translation_unit.diagnostics:
            for diagnostic in translation_unit.diagnostics:
                if diagnostic.severity >= clang.Diagnostic.Error:
                    # logging.warning(diagnostic.spelling)
                    print("WARNING" + diagnostic.spelling)

    def _cursor_obj(self, cursor):
        line = 0
        declaration_file_path = ""
        var_type = ""
        line_tokens = []
        try:
            if cursor.location.file:
                line = cursor.location.line
        except AttributeError as e:
            pass
        name = repr(cursor.kind)[11:]
        spell = cursor.spelling or "[no name]"

        try:
            declaration_file_path = cursor.referenced.location.file.name
        except AttributeError as e:
            print(spell + ": declaration_file_name not present")

        try:
            var_type = cursor.type.spelling
        except AttributeError:
            print(spell + ": var_type not present")

        try:
            token_list = list(cursor.get_tokens())
            for i in range(0, len(token_list)):
                line_tokens.append(token_list[i].spelling)

            #print(name + " : " + spell + ":  " + str(line_tokens))
        except AttributeError as e:
            print("Couldn't get line_tokens")

        return {
            AstConstants.LINE: line,
            AstConstants.KIND: name,
            AstConstants.NAME: spell,
            AstConstants.DECL_FILEPATH: declaration_file_path,
            AstConstants.VAR_TYPE: var_type,
            AstConstants.LINE_TOKENS: line_tokens
        }

    def _cursor_str(self, cursor, indent):
        line = 0
        col = 0
        try:
            if cursor.location.file:
                line = cursor.location.line
                col = cursor.location.column
        except ArgumentError as e:
            pass
        name = repr(cursor.kind)[11:]
        spell = cursor.spelling or "[no spelling]"
        tokens = len(list(cursor.get_tokens()))
        prefix = indent * "| "
        return "{}[{}:{}] {}: {} [{} tokens]".format(prefix, line, col,
                                                    name, spell, tokens)


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
