
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
# Notes
###############################################################################

# This is not really an implementation. It is more or less a template,
# extracted from the clang cpp implementation, as an inspiration for other
# implementations.

###############################################################################
# Imports
###############################################################################

from .model import (
    CodeExpression, CodeExpressionStatement, CodeVariable, CodeGlobalScope
)


###############################################################################
# Language Entity Builders
###############################################################################

class CodeEntityBuilder(object):
    def __init__(self, scope, parent):
        self.scope  = scope
        self.parent = parent
        self.file   = None
        self.line   = None
        self.column = None

    def build(self, data):
        """Build an object for the current tree node and
            corresponding builders for the node's children.
            Return None if an object cannot be built.
            Return (object, [builders]) otherwise.
        """
        return None

    def _lookup_parent(self, cls):
        codeobj = self.parent
        while not codeobj is None and not isinstance(codeobj, cls):
            codeobj = codeobj.parent
        return codeobj



class CodeExpressionBuilder(CodeEntityBuilder):
    def __init__(self, scope, parent):
        CodeEntityBuilder.__init__(self, scope, parent)

    def build(self, data):
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
        return result


    def _build_literal(self):
        return None

    def _build_reference(self, data):
        return None

    def _build_operator(self):
        return None

    def _build_function_call(self, data):
        return None

    def _build_default_argument(self):
        return None

    def _build_other(self, data):
        return None


class CodeStatementBuilder(CodeEntityBuilder):
    def __init__(self, scope, parent):
        CodeEntityBuilder.__init__(self, scope, parent)

    def build(self, data):
        result = self._build_declarations(data)
        result = result or self._build_expression(data)
        result = result or self._build_control_flow()
        result = result or self._build_jump_statement()
        result = result or self._build_block()
        return result


    def _build_expression(self, data):
        builder = CodeExpressionBuilder(self.scope, self.parent)
        result = builder.build(data)
        if result:
            expression = result[0]
            codeobj = CodeExpressionStatement(self.scope, self.parent,
                                              expression = expression)
            codeobj.file = self.file
            codeobj.line = self.line
            codeobj.column = self.column
            if isinstance(expression, CodeExpression):
                expression.parent = codeobj
            result = (codeobj, result[1])
        return result

    def _build_declarations(self, data):
        return None

    def _build_control_flow(self):
        return None

    def _build_jump_statement(self):
        return None

    def _build_block(self):
        return None



class CodeTopLevelBuilder(CodeEntityBuilder):
    def __init__(self, scope, parent, workspace = ""):
        CodeEntityBuilder.__init__(self, scope, parent)
        self.workspace = workspace

    def build(self, data):
        result = self._build_variable(data)
        result = result or self._build_function(data)
        result = result or self._build_class(data)
        result = result or self._build_namespace()
        return result


    def _build_variable(self, data):
        return None

    def _build_function(self, data):
        return None

    def _build_class(self, data):
        return None

    def _build_namespace(self):
        return None



###############################################################################
# AST Parsing
###############################################################################

class MultipleDefinitionError(Exception):
    pass

class AnalysisData(object):
    def __init__(self):
        self.entities   = {}    # id -> CodeEntity
        self._refs      = {}    # id -> [CodeEntity]

    def register(self, codeobj, declaration = False):
        previous = self.entities.get(codeobj.id)
        if declaration and not previous is None:
            codeobj._definition = previous
            return
        if not declaration and not previous is None:
            if not isinstance(codeobj, CodeVariable):
                assert not isinstance(previous, CodeVariable)
                if previous.is_definition:
                    raise MultipleDefinitionError("Multiple definitions for "
                                                  + codeobj.name)
                previous._definition = codeobj
            for ref in previous.references:
                codeobj.references.append(ref)
                ref.reference = codeobj
            previous.references = []
        self.entities[codeobj.id] = codeobj
        if codeobj.id in self._refs:
            for ref in self._refs[codeobj.id]:
                codeobj.references.append(ref)
                ref.reference = codeobj
            del self._refs[codeobj.id]

    def reference(self, id, ref):
        codeobj = self.entities.get(id)
        if not codeobj is None:
            codeobj.references.append(ref)
            ref.reference = codeobj
        else:
            if not id in self._refs:
                self._refs[id] = []
            self._refs[id].append(ref)
            ref.reference = id


class CodeAstParser(object):
    def __init__(self, workspace = ""):
        self.workspace      = workspace
        self.global_scope   = CodeGlobalScope()
        self.data           = AnalysisData()

    def parse(self, file_path):
        return self.global_scope
