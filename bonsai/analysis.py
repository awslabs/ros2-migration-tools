
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

from .model import (
    CodeEntity, CodeBlock, CodeControlFlow, CodeExpression, CodeFunction,
    CodeFunctionCall, CodeOperator, CodeReference, CodeVariable, CodeLoop,
    CodeDefaultArgument
)


###############################################################################
# AST Analysis
###############################################################################

class CodeQuery(object):
    def __init__(self, codeobj):
        assert isinstance(codeobj, CodeEntity)
        self.root = codeobj
        self.cls = None
        self.recursive = False
        self.attributes = {}

    @property
    def references(self):
        self.cls = CodeReference
        self.recursive = False
        return self

    @property
    def all_references(self):
        self.cls = CodeReference
        self.recursive = True
        return self

    @property
    def calls(self):
        self.cls = CodeFunctionCall
        self.recursive = False
        return self

    @property
    def all_calls(self):
        self.cls = CodeFunctionCall
        self.recursive = True
        return self

    def where_name(self, name):
        self.attributes["name"] = name
        return self

    def where_result(self, result):
        self.attributes["result"] = result
        return self

    def get(self):
        result = []
        for codeobj in self.root.filter(self.cls, recursive = self.recursive):
            passes = True
            for key, value in self.attributes.iteritems():
                if isinstance(value, basestring):
                    if getattr(codeobj, key) != value:
                        passes = False
                else:
                    if not getattr(codeobj, key) in value:
                        passes = False
            if passes:
                result.append(codeobj)
        return result


###############################################################################
# Interface Functions
###############################################################################

def resolve_expression(expression):
    assert isinstance(expression, CodeExpression.TYPES)
    if isinstance(expression, CodeReference):
        return resolve_reference(expression)
    if isinstance(expression, CodeOperator):
        args = []
        for arg in expression.arguments:
            arg = resolve_expression(arg)
            if not isinstance(arg, CodeExpression.LITERALS):
                return expression
            args.append(arg)
        if expression.is_binary:
            a = args[0]
            b = args[1]
            if not isinstance(a, CodeExpression.LITERALS) \
                    or not isinstance(b, CodeExpression.LITERALS):
                return expression
            if expression.name == "+":
                return a + b
            if expression.name == "-":
                return a - b
            if expression.name == "*":
                return a * b
            if expression.name == "/":
                return a / b
            if expression.name == "%":
                return a % b
    # if isinstance(expression, CodeExpression.LITERALS):
    # if isinstance(expression, SomeValue):
    # if isinstance(expression, CodeFunctionCall):
    # if isinstance(expression, CodeDefaultArgument):
    return expression


def resolve_reference(reference):
    assert isinstance(reference, CodeReference)
    if reference.statement is None:
        return None # TODO investigate
    si = reference.statement._si
    if (reference.reference is None
            or isinstance(reference.reference, basestring)):
        return None
    if isinstance(reference.reference, CodeVariable):
        var = reference.reference
        value = var.value
        function = reference.function
        for w in var.writes:
            ws = w.statement
            if not w.function is function:
                continue
            if ws._si < si:
                if w.arguments[0].reference is var:
                    value = resolve_expression(w.arguments[1])
                else:
                    continue # TODO
            elif ws._si == si:
                if w.arguments[0] is reference:
                    value = resolve_expression(w.arguments[1])
                else:
                    continue # TODO
        if value is None:
            if var.is_parameter:
                calls = [call for call in function.references
                         if isinstance(call, CodeFunctionCall)]
                if len(calls) != 1:
                    return None
                i = function.parameters.index(var)
                if len(calls[0].arguments) <= i:
                    return None
                arg = calls[0].arguments[i]
                if isinstance(arg, CodeReference):
                    return resolve_reference(arg)
                return arg
            if (function.is_constructor and var.member_of is not None
                    and function.member_of is var.member_of):
                # variable is an auto-initialised member of the class
                return var.auto_init()
        if isinstance(value, CodeExpression.TYPES):
            return resolve_expression(value)
        return value
    return reference.reference


def is_under_control_flow(codeobj, recursive = False):
    return get_control_depth(codeobj, recursive) > 0


def get_control_depth(codeobj, recursive = False):
    depth = 0
    while not codeobj is None:
        if (isinstance(codeobj, CodeBlock)
                and isinstance(codeobj.parent, CodeControlFlow)):
            depth += 1
        elif isinstance(codeobj, CodeFunction):
            if recursive:
                calls = [get_control_depth(call) for call in codeobj.references
                                if isinstance(call, CodeFunctionCall)]
                if calls:
                    depth += max(calls)
            return depth
        codeobj = codeobj.parent
    return depth


def is_under_loop(codeobj, recursive = False):
    while not codeobj is None:
        if (isinstance(codeobj, CodeBlock)
                and isinstance(codeobj.parent, CodeLoop)):
            return True
        elif isinstance(codeobj, CodeFunction):
            if recursive:
                return any(is_under_loop(call)
                           for call in codeobj.references
                           if isinstance(call, CodeFunctionCall))
            return False
        codeobj = codeobj.parent
    return False


def get_conditions(codeobj, recursive = False):
    conditions = []
    while not codeobj is None:
        if (isinstance(codeobj, CodeBlock)
                and isinstance(codeobj.parent, CodeControlFlow)):
            conditions.append(codeobj.parent.condition)
        elif isinstance(codeobj, CodeFunction):
            if recursive:
                for call in codeobj.references:
                    if isinstance(call, CodeFunctionCall):
                        conditions.extend(get_conditions(call))
            return conditions
        codeobj = codeobj.parent
    return conditions
