
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
# Language Model
###############################################################################


class CodeEntity(object):
    """Base class for all programming entities.

        All code objects have a file name, a line number, a column number,
        a programming scope (e.g. the function or code block they belong to)
        and a parent object that should have some variable or collection
        holding this object.
    """

    def __init__(self, scope, parent):
        """Base constructor for code objects.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
        """
        self.scope = scope
        self.parent = parent
        self.file = None
        self.line = None
        self.column = None

    def walk_preorder(self):
        """Iterates the program tree starting from this object, going down."""
        yield self
        for child in self._children():
            for descendant in child.walk_preorder():
                yield descendant

    def filter(self, cls, recursive=False):
        """Retrieves all descendants (including self) that are instances
            of a given class.

        Args:
            cls (class): The class to use as a filter.

        Kwargs:
            recursive (bool): Whether to descend recursively down the tree.
        """
        source = self.walk_preorder if recursive else self._children
        return [
            codeobj
            for codeobj in source()
            if isinstance(codeobj, cls)
        ]

    def _afterpass(self):
        """Finalizes the construction of a code entity."""
        pass

    def _validity_check(self):
        """Check whether this object is a valid construct."""
        return True

    def _children(self):
        """Yield all direct children of this object."""
        # The default implementation has no children, and thus should return
        # an empty iterator.
        return iter(())

    def _lookup_parent(self, cls):
        """Lookup a transitive parent object that is an instance
            of a given class."""
        codeobj = self.parent
        while codeobj is not None and not isinstance(codeobj, cls):
            codeobj = codeobj.parent
        return codeobj

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        return (' ' * indent) + self.__str__()

    def ast_str(self, indent=0):
        """Return a minimal string to print a tree-like structure.

        Kwargs:
            indent (int): The number of indentation levels.
        """
        line = self.line or 0
        col = self.column or 0
        name = type(self).__name__
        spell = getattr(self, 'name', '[no spelling]')
        result = ' ({})'.format(self.result) if hasattr(self, 'result') else ''
        prefix = indent * '| '
        return '{}[{}:{}] {}{}: {}'.format(prefix, line, col,
                                           name, result, spell)

    def __str__(self):
        """Return a string representation of this object."""
        return self.__repr__()

    def __repr__(self):
        """Return a string representation of this object."""
        return '[unknown]'


class CodeStatementGroup(object):
    """This class is meant to provide common utility methods for
        objects that group multiple program statements together
        (e.g. functions, code blocks).

        It is not meant to be instantiated directly, only used for
        inheritance purposes.

        It defines the length of a statement group, and provides methods
        for integer-based indexing of program statements (as if using a list).
    """

    def statement(self, i):
        """Return the *i*-th statement from the object's `body`."""
        return self.body.statement(i)

    def statement_after(self, i):
        """Return the statement after the *i*-th one, or `None`."""
        try:
            return self.statement(i + 1)
        except IndexError as e:
            return None

    def __getitem__(self, i):
        """Return the *i*-th statement from the object's `body`."""
        return self.statement(i)

    def __len__(self):
        """Return the length of the statement group."""
        return len(self.body)


# ----- Common Entities -------------------------------------------------------

class CodeVariable(CodeEntity):
    """This class represents a program variable.

        A variable typically has a name, a type (`result`) and a value
        (or `None` for variables without a value or when the value is unknown).

        Additionally, a variable has an `id` which uniquely identifies it in
        the program (useful to resolve references), a list of references to it
        and a list of statements that write new values to the variable.

        If the variable is a *member*/*field*/*attribute* of an object,
        `member_of` should contain a reference to such object, instead of `None`.
    """

    def __init__(self, scope, parent, id, name, result):
        """Constructor for variables.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            id: An unique identifier for this variable.
            name (str): The name of the variable in the program.
            result (str): The type of the variable in the program.
        """
        CodeEntity.__init__(self, scope, parent)
        self.id = id
        self.name = name
        self.result = result
        self.value = None
        self.member_of = None
        self.references = []
        self.writes = []

    @property
    def is_definition(self):
        return True

    @property
    def is_local(self):
        """Whether this is a local variable.

            In general, a variable is *local* if its containing scope is a
            statement (e.g. a block), or a function, given that the variable
            is not one of the function's parameters.
        """
        return (isinstance(self.scope, CodeStatement)
                or (isinstance(self.scope, CodeFunction)
                    and self not in self.scope.parameters))

    @property
    def is_global(self):
        """Whether this is a global variable.

            In general, a variable is *global* if it is declared directly under
            the program's global scope or a namespace.
        """
        return isinstance(self.scope, (CodeGlobalScope, CodeNamespace))

    @property
    def is_parameter(self):
        """Whether this is a function parameter."""
        return (isinstance(self.scope, CodeFunction)
                and self in self.scope.parameters)

    @property
    def is_member(self):
        """Whether this is a member/attribute of a class or object."""
        return isinstance(self.scope, CodeClass)

    def auto_init(self):
        """Return a default value for this variable."""
        assign = CodeOperator(self.scope, self.parent, "=", self.result)
        value = CodeDefaultArgument(self.scope, assign, self.result)
        assign.arguments = (self, value)
        return value

    def _add(self, codeobj):
        """Add a child (value) to this object."""
        assert isinstance(codeobj, CodeExpression.TYPES)
        self.value = codeobj

    def _children(self):
        """Yield all direct children of this object."""
        if isinstance(self.value, CodeEntity):
            yield self.value

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        return '{}{} {} = {}'.format(' ' * indent, self.result, self.name,
                                     pretty_str(self.value))

    def __repr__(self):
        """Return a string representation of this object."""
        return '[{}] {} = ({})'.format(self.result, self.name, self.value)


class CodeFunction(CodeEntity, CodeStatementGroup):
    """This class represents a program function.

        A function typically has a name, a return type (`result`), a list
        of parameters and a body (a code block). It also has an unique `id`
        that identifies it in the program and a list of references to it.

        If a function is a method of some class, its `member_of` should be
        set to the corresponding class.
    """

    def __init__(self, scope, parent, id, name, result, definition=True):
        """Constructor for functions.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            id: An unique identifier for this function.
            name (str): The name of the function in the program.
            result (str): The return type of the function in the program.
        """
        CodeEntity.__init__(self, scope, parent)
        self.id = id
        self.name = name
        self.result = result
        self.parameters = []
        self.body = CodeBlock(self, self, explicit=True)
        self.member_of = None
        self.references = []
        self._definition = self if definition else None

    @property
    def is_definition(self):
        """Whether this is a function definition or just a declaration."""
        return self._definition is self

    @property
    def is_constructor(self):
        """Whether this function is a class constructor."""
        return self.member_of is not None

    def _add(self, codeobj):
        """Add a child (statement) to this object."""
        assert isinstance(codeobj, (CodeStatement, CodeExpression))
        self.body._add(codeobj)

    def _children(self):
        """Yield all direct children of this object."""
        for codeobj in self.parameters:
            yield codeobj
        for codeobj in self.body._children():
            yield codeobj

    def _afterpass(self):
        """Assign a function-local index to each child object and register
            write operations to variables.

            This should only be called after the object is fully built.
        """
        if hasattr(self, '_fi'):
            return
        fi = 0
        for codeobj in self.walk_preorder():
            codeobj._fi = fi
            fi += 1
            if isinstance(codeobj, CodeOperator) and codeobj.is_assignment:
                if codeobj.arguments and isinstance(codeobj.arguments[0],
                                                    CodeReference):
                    var = codeobj.arguments[0].reference
                    if isinstance(var, CodeVariable):
                        var.writes.append(codeobj)

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        spaces = ' ' * indent
        params = ', '.join(map(lambda p: p.result + ' ' + p.name,
                           self.parameters))
        if self.is_constructor:
            pretty = '{}{}({}):\n'.format(spaces, self.name, params)
        else:
            pretty = '{}{} {}({}):\n'.format(spaces, self.result,
                                             self.name, params)
        if self._definition is not self:
            pretty += spaces + '  [declaration]'
        else:
            pretty += self.body.pretty_str(indent + 2)
        return pretty

    def __repr__(self):
        """Return a string representation of this object."""
        params = ', '.join(map(str, self.parameters))
        return '[{}] {}({})'.format(self.result, self.name, params)


class CodeClass(CodeEntity):
    """This class represents a program class for object-oriented languages.

        A class typically has a name, an unique `id`, a list of
        members (variables, functions), a list of superclasses, and a list of
        references.

        If a class is defined within another class (inner class), it should
        have its `member_of` set to the corresponding class.
    """

    def __init__(self, scope, parent, id_, name, definition=True):
        """Constructor for classes.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            id: An unique identifier for this class.
            name (str): The name of the class in the program.
        """
        CodeEntity.__init__(self, scope, parent)
        self.id = id_
        self.name = name
        self.members = []
        self.superclasses = []
        self.member_of = None
        self.references = []
        self._definition = self if definition else None

    @property
    def is_definition(self):
        """Whether this is a definition or a declaration of the class."""
        return self._definition is self

    def _add(self, codeobj):
        """Add a child (function, variable, class) to this object."""
        assert isinstance(codeobj, (CodeFunction, CodeVariable, CodeClass))
        self.members.append(codeobj)
        codeobj.member_of = self

    def _children(self):
        """Yield all direct children of this object."""
        for codeobj in self.members:
            yield codeobj

    def _afterpass(self):
        """Assign the `member_of` of child members and call
            their `_afterpass()`.

            This should only be called after the object is fully built.
        """
        for codeobj in self.members:
            if not codeobj.is_definition:
                if not codeobj._definition is None:
                    codeobj._definition.member_of = self
            codeobj._afterpass()

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        spaces = ' ' * indent
        pretty = spaces + 'class ' + self.name
        if self.superclasses:
            superclasses = ', '.join(self.superclasses)
            pretty += '(' + superclasses + ')'
        pretty += ':\n'
        if self.members:
            pretty += '\n\n'.join(
                    c.pretty_str(indent + 2)
                    for c in self.members
            )
        else:
            pretty += spaces + '  [declaration]'
        return pretty

    def __repr__(self):
        """Return a string representation of this object."""
        return '[class {}]'.format(self.name)


class CodeNamespace(CodeEntity):
    """This class represents a program namespace.

        A namespace is a concept that is explicit in languages such as C++,
        but less explicit in many others. In Python, the closest thing should
        be a module. In Java, it may be the same as a class, or non-existent.

        A namespace typically has a name and a list of children objects
        (variables, functions or classes).
    """

    def __init__(self, scope, parent, name):
        """Constructor for namespaces.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            name (str): The name of the namespace in the program.
        """
        CodeEntity.__init__(self, scope, parent)
        self.name = name
        self.children = []

    def _add(self, codeobj):
        """Add a child (namespace, function, variable, class) to this object."""
        assert isinstance(codeobj, (CodeNamespace, CodeClass,
                                    CodeFunction, CodeVariable))
        self.children.append(codeobj)

    def _children(self):
        """Yield all direct children of this object."""
        for codeobj in self.children:
            yield codeobj

    def _afterpass(self):
        """Call the `_afterpass()` of child objects.

            This should only be called after the object is fully built.
        """
        for codeobj in self.children:
            codeobj._afterpass()

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        spaces = ' ' * indent
        pretty = '{}namespace {}:\n'.format(spaces, self.name)
        pretty += '\n\n'.join(c.pretty_str(indent + 2) for c in self.children)
        return pretty

    def __repr__(self):
        """Return a string representation of this object."""
        return '[namespace {}]'.format(self.name)


class CodeGlobalScope(CodeEntity):
    """This class represents the global scope of a program.

        The global scope is the root object of a program. If there are no
        better candidates, it is the `scope` and `parent` of all other objects.
        It is also the only object that does not have a `scope` or `parent`.
    """

    def __init__(self):
        """Constructor for global scope objects."""
        CodeEntity.__init__(self, None, None)
        self.children = []

    def _add(self, codeobj):
        """Add a child (namespace, function, variable, class) to this object."""
        assert isinstance(codeobj, (CodeNamespace, CodeClass,
                                    CodeFunction, CodeVariable))
        self.children.append(codeobj)

    def _children(self):
        """Yield all direct children of this object."""
        for codeobj in self.children:
            yield codeobj

    def _afterpass(self):
        """Call the `_afterpass()` of child objects.

            This should only be called after the object is fully built.
        """
        for codeobj in self.children:
            codeobj._afterpass()

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        return '\n\n'.join(
                codeobj.pretty_str(indent=indent)
                for codeobj in self.children
        )

    def __repr__(self):
        """Return a string representation of this object."""
        return 'CodeGlobalScope({})'.format(self.children)


# ----- Expression Entities ---------------------------------------------------

class CodeExpression(CodeEntity):
    """Base class for expressions within a program.

        Expressions can be of many types, including literal values,
        operators, references and function calls. This class is meant
        to be inherited from, and not instantiated directly.

        An expression typically has a name (e.g. the name of the function
        in a function call) and a type (`result`). Also, an expression should
        indicate whether it is enclosed in parentheses.
    """

    def __init__(self, scope, parent, name, result, paren=False):
        """Constructor for expressions.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            name (str): The name of the expression in the program.
            result (str): The return type of the expression in the program.

        Kwargs:
            paren (bool): Whether the expression is enclosed in parentheses.
        """
        CodeEntity.__init__(self, scope, parent)
        self.name = name
        self.result = result
        self.parenthesis = paren

    @property
    def function(self):
        """The function where this expression occurs."""
        return self._lookup_parent(CodeFunction)

    @property
    def statement(self):
        """The statement where this expression occurs."""
        return self._lookup_parent(CodeStatement)

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        if self.parenthesis:
            return (' ' * indent) + '(' + self.name + ')'
        return (' ' * indent) + self.name

    def __repr__(self):
        """Return a string representation of this object."""
        return '[{}] {}'.format(self.result, self.name)


class SomeValue(CodeExpression):
    """This class represents an unknown value for diverse primitive types."""

    def __init__(self, result):
        """Constructor for unknown values."""
        CodeExpression.__init__(self, None, None, result, result)

    def _children(self):
        """Yield all the children of this object, that is no children."""
        return iter(())

SomeValue.INTEGER = SomeValue("int")
SomeValue.FLOATING = SomeValue("float")
SomeValue.CHARACTER = SomeValue("char")
SomeValue.STRING = SomeValue("string")
SomeValue.BOOL = SomeValue("bool")


class CodeLiteral(CodeExpression):
    """Base class for literal types not present in Python.

        This class is meant to represent a literal whose type is not numeric,
        string or boolean, as bare Python literals are used for those.

        A literal has a value (e.g. a list `[1, 2, 3]`) and a type (`result`),
        and could be enclosed in parentheses. It does not have a name.
    """

    def __init__(self, scope, parent, value, result, paren=False):
        """Constructor for literals.

            As literals have no name, a constant string is used instead.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            value (CodeExpression|CodeExpression[]): This literal's value.
            result (str): The return type of the literal in the program.

        Kwargs:
            paren (bool): Whether the literal is enclosed in parentheses.
        """
        CodeExpression.__init__(self, scope, parent, 'literal', result, paren)
        self.value = value

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        if self.parenthesis:
            return '{}({})'.format(' ' * indent, pretty_str(self.value))
        return pretty_str(self.value, indent=indent)

    def __repr__(self):
        """Return a string representation of this object."""
        return '[{}] {!r}'.format(self.result, self.value)


CodeExpression.TYPES = (int, int, float, bool, str, SomeValue,
                        CodeLiteral, CodeExpression)

CodeExpression.LITERALS = (int, int, float, bool, str, CodeLiteral)


class CodeNull(CodeLiteral):
    """This class represents an indefinite value.

        Many programming languages have their own version of this concept:
        Java has null references, C/C++ NULL pointers, Python None and so on.
    """

    def __init__(self, scope, parent, paren=False):
        """Constructor for null literals.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.

        Kwargs:
            paren (bool): Whether the null literal is enclosed in parentheses.
        """
        CodeLiteral.__init__(self, scope, parent, None, 'null', paren)

    def _children(self):
        """Yield all the children of this object, that is no children.

        This class inherits from CodeLiteral just for consistency with the
        class hierarchy. It should have no children, thus an empty iterator
        is returned.
        """
        return iter(())


class CodeCompositeLiteral(CodeLiteral):
    """This class represents a composite literal.

        A composite literal is any type of literal whose value is compound,
        rather than simple. An example present in many programming languages
        are list literals, often constructed as `[1, 2, 3]`.

        A composite literal has a sequence of values that compose it
        (`values`), a type (`result`), and it should indicate whether it is
        enclosed in parentheses.

    """

    def __init__(self, scope, parent, result, value=(), paren=False):
        """Constructor for a compound literal.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            value (iterable): The initial value sequence in this composition.
            result (str): The return type of the literal in the program.

        Kwargs:
            paren (bool): Whether the literal is enclosed in parentheses.

        """
        try:
            value = list(value)
        except TypeError as te:
            raise AssertionError(str(te))

        CodeLiteral.__init__(self, scope, parent, value, result, paren)

    @property
    def values(self):
        return tuple(self.value)

    def _add_value(self, child):
        """Add a value to the sequence in this composition."""
        self.value.append(child)

    def _children(self):
        """Yield all direct children of this object."""
        for value in self.value:
            if isinstance(value, CodeEntity):
                yield value

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        indent = ' ' * indent
        values = '{{{}}}'.format(', '.join(map(pretty_str, self.value)))

        if self.parenthesis:
            return '{}({})'.format(indent, values)
        return '{}{}'.format(indent, values)

    def __repr__(self):
        """Return a string representation of this object."""
        return '[{}] {{{}}}'.format(self.result,
                                    ', '.join(map(repr, self.value)))


class CodeReference(CodeExpression):
    """This class represents a reference expression (e.g. to a variable).

        A reference typically has a name (of what it is referencing),
        and a return type.

        If the referenced entity is known, `reference` should be set.

        If the reference is a field/attribute of an object, `field_of`
        should be set to that object.
    """

    def __init__(self, scope, parent, name, result, paren=False):
        """Constructor for references.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            name (str): The name of the reference in the program.
            result (str): The return type of the expression in the program.

        Kwargs:
            paren (bool): Whether the reference is enclosed in parentheses.
        """
        CodeExpression.__init__(self, scope, parent, name, result, paren)
        self.field_of = None
        self.reference = None

    def _set_field(self, codeobj):
        """Set the object that contains the attribute this is a reference of."""
        assert isinstance(codeobj, CodeExpression)
        self.field_of = codeobj

    def _children(self):
        """Yield all direct children of this object."""
        if self.field_of:
            yield self.field_of

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        spaces = ' ' * indent
        pretty = '{}({})' if self.parenthesis else '{}{}'
        name = ('{}.{}'.format(self.field_of.pretty_str(), self.name)
                if self.field_of else self.name)
        return pretty.format(spaces, name)

    def __str__(self):
        """Return a string representation of this object."""
        return '#' + self.name

    def __repr__(self):
        """Return a string representation of this object."""
        if self.field_of:
            return '[{}] ({}).{}'.format(self.result, self.field_of, self.name)
        return '[{}] #{}'.format(self.result, self.name)


class CodeOperator(CodeExpression):
    """This class represents an operator expression (e.g. `a + b`).

        Operators can be unary or binary, and often return numbers
        or booleans. Some languages also support ternary operators.

        Do note that assignments are often considered expressions,
        and, as such, assignment operators are included here.

        An operator typically has a name (its token), a return type,
        and a tuple of its arguments.
    """

    _UNARY_TOKENS = ("+", "-")

    _BINARY_TOKENS = ("+", "-", "*", "/", "%", "<", ">", "<=", ">=",
                      "==", "!=", "&&", "||", "=")

    def __init__(self, scope, parent, name, result, args=None, paren=False):
        """Constructor for operators.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            name (str): The name of the operator in the program.
            result (str): The return type of the operator in the program.

        Kwargs:
            args (tuple): Initial tuple of arguments.
            paren (bool): Whether the expression is enclosed in parentheses.
        """
        CodeExpression.__init__(self, scope, parent, name, result, paren)
        self.arguments = args or ()

    @property
    def is_unary(self):
        """Whether this is a unary operator."""
        return len(self.arguments) == 1

    @property
    def is_binary(self):
        """Whether this is a binary operator."""
        return len(self.arguments) == 2

    @property
    def is_ternary(self):
        """Whether this is a ternary operator."""
        return len(self.arguments) == 3

    @property
    def is_assignment(self):
        """Whether this is an assignment operator."""
        return self.name == "="

    def _add(self, codeobj):
        """Add a child (argument) to this object."""
        assert isinstance(codeobj, CodeExpression.TYPES)
        self.arguments = self.arguments + (codeobj,)

    def _children(self):
        """Yield all direct children of this object."""
        for codeobj in self.arguments:
            if isinstance(codeobj, CodeExpression):
                yield codeobj

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        indent = ' ' * indent
        pretty = '{}({})' if self.parenthesis else '{}{}'
        if self.is_unary:
            operator = self.name + pretty_str(self.arguments[0])
        else:
            operator = '{} {} {}'.format(pretty_str(self.arguments[0]),
                                         self.name,
                                         pretty_str(self.arguments[1]))
        return pretty.format(indent, operator)

    def __repr__(self):
        """Return a string representation of this object."""
        if self.is_unary:
            return '[{}] {}({})'.format(self.result, self.name,
                                        self.arguments[0])
        if self.is_binary:
            return '[{}] ({}){}({})'.format(self.result, self.arguments[0],
                                            self.name, self.arguments[1])
        return '[{}] {}'.format(self.result, self.name)


class CodeFunctionCall(CodeExpression):
    """This class represents a function call.

        A function call typically has a name (of the called function),
        a return type, a tuple of its arguments and a reference to the
        called function.

        If a call references a class method, its `method_of` should be
        set to the object on which a method is being called.
    """

    def __init__(self, scope, parent, name, result, paren=False):
        """Constructor for function calls.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            name (str): The name of the function in the program.
            result (str): The return type of the expression in the program.

        Kwargs:
            paren (bool): Whether the expression is enclosed in parentheses.
        """
        CodeExpression.__init__(self, scope, parent, name, result, paren)
        self.full_name = name
        self.arguments = ()
        self.method_of = None
        self.reference = None

    @property
    def is_constructor(self):
        """Whether the called function is a constructor."""
        return self.result == self.name

    def _add(self, codeobj):
        """Add a child (argument) to this object."""
        assert isinstance(codeobj, CodeExpression.TYPES)
        self.arguments = self.arguments + (codeobj,)

    def _set_method(self, codeobj):
        """Set the object on which a method is called."""
        assert isinstance(codeobj, CodeExpression)
        self.method_of = codeobj

    def _children(self):
        """Yield all direct children of this object."""
        if self.method_of:
            yield self.method_of
        for codeobj in self.arguments:
            if isinstance(codeobj, CodeExpression):
                yield codeobj

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        indent = ' ' * indent
        pretty = '{}({})' if self.parenthesis else '{}{}'
        args = ', '.join(map(pretty_str, self.arguments))
        if self.method_of:
            call = '{}.{}({})'.format(self.method_of.pretty_str(),
                                      self.name, args)
        elif self.is_constructor:
            call = 'new {}({})'.format(self.name, args)
        else:
            call = '{}({})'.format(self.name, args)
        return pretty.format(indent, call)

    def __repr__(self):
        """Return a string representation of this object."""
        args = ', '.join(map(str, self.arguments))
        if self.is_constructor:
            return '[{}] new {}({})'.format(self.result, self.name, args)
        if self.method_of:
            return '[{}] {}.{}({})'.format(self.result, self.method_of.name,
                                           self.name, args)
        return '[{}] {}({})'.format(self.result, self.name, args)


class CodeDefaultArgument(CodeExpression):
    """This class represents a default argument.

        Some languages, such as C++, allow function parameters to have
        default values when not explicitly provided by the programmer.
        This class represents such omitted arguments.

        A default argument has only a return type.
    """

    def __init__(self, scope, parent, result):
        """Constructor for default arguments.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            result (str): The return type of the argument in the program.
        """
        CodeExpression.__init__(self, scope, parent, '(default)', result)


# ----- Statement Entities ----------------------------------------------------

class CodeStatement(CodeEntity):
    """Base class for program statements.

        Programming languages often define diverse types of statements
        (e.g. return statements, control flow, etc.).
        This class provides common functionality for such statements.
        In many languages, statements must be contained within a function.

        An operator typically has a name (its token), a return type,
        and a tuple of its arguments.
    """

    def __init__(self, scope, parent):
        """Constructor for statements.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
        """
        CodeEntity.__init__(self, scope, parent)
        self._si = -1

    @property
    def function(self):
        """The function where this statement appears in."""
        return self._lookup_parent(CodeFunction)


class CodeJumpStatement(CodeStatement):
    """This class represents a jump statement (e.g. `return`, `break`).

        A jump statement has a name. In some cases, it may also have an
        associated value (e.g. `return 0`).
    """

    def __init__(self, scope, parent, name):
        """Constructor for jump statements.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            name (str): The name of the statement in the program.
        """
        CodeStatement.__init__(self, scope, parent)
        self.name = name
        self.value = None

    def _add(self, codeobj):
        """Add a child (value) to this object."""
        assert isinstance(codeobj, CodeExpression.TYPES)
        self.value = codeobj

    def _children(self):
        """Yield all direct children of this object."""
        if isinstance(self.value, CodeExpression):
            yield self.value

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        indent = ' ' * indent
        if self.value is not None:
            return '{}{} {}'.format(indent, self.name, pretty_str(self.value))
        return indent + self.name

    def __repr__(self):
        """Return a string representation of this object."""
        if self.value is not None:
            return '{} {}'.format(self.name, str(self.value))
        return self.name


class CodeExpressionStatement(CodeStatement):
    """This class represents an expression statement. It is only a wrapper.

        Many programming languages allow expressions to be statements
        on their own. A common example is the assignment operator, which
        can be a statement on its own, but also returns a value when
        contained within a larger expression.
    """

    def __init__(self, scope, parent, expression=None):
        """Constructor for expression statements.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.

        Kwargs:
            expression (CodeExpression): The expression of this statement.
        """
        CodeStatement.__init__(self, scope, parent)
        self.expression = expression

    def _children(self):
        """Yield all direct children of this object."""
        if isinstance(self.expression, CodeExpression):
            yield self.expression

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        return pretty_str(self.expression, indent=indent)

    def __repr__(self):
        """Return a string representation of this object."""
        return repr(self.expression)


class CodeBlock(CodeStatement, CodeStatementGroup):
    """This class represents a code block (e.g. `{}` in C, C++, Java, etc.).

        Blocks are little more than collections of statements, while being
        considered a statement themselves.

        Some languages allow blocks to be implicit in some contexts, e.g.
        an `if` statement omitting curly braces in C, C++, Java, etc.

        This model assumes that control flow branches and functions always
        have a block as their body.
    """

    def __init__(self, scope, parent, explicit=True):
        """Constructor for code blocks.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.

        Kwargs:
            explicit (bool): Whether the block is explicit in the code.
        """
        CodeStatement.__init__(self, scope, parent)
        self.body = []
        self.explicit = explicit

    def statement(self, i):
        """Return the *i*-th statement of this block."""
        return self.body[i]

    def _add(self, codeobj):
        """Add a child (statement) to this object."""
        assert isinstance(codeobj, CodeStatement)
        codeobj._si = len(self.body)
        self.body.append(codeobj)

    def _children(self):
        """Yield all direct children of this object."""
        for codeobj in self.body:
            yield codeobj

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        if self.body:
            return '\n'.join(stmt.pretty_str(indent) for stmt in self.body)
        else:
            return (' ' * indent) + '[empty]'

    def __repr__(self):
        """Return a string representation of this object."""
        return str(self.body)


class CodeDeclaration(CodeStatement):
    """This class represents a declaration statement.

        Some languages, such as C, C++ or Java, consider this special
        kind of statement for declaring variables within a function,
        for instance.

        A declaration statement contains a list of all declared variables.
    """

    def __init__(self, scope, parent):
        """Constructor for declaration statements.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
        """
        CodeStatement.__init__(self, scope, parent)
        self.variables = []

    def _add(self, codeobj):
        """Add a child (variable) to this object."""
        assert isinstance(codeobj, CodeVariable)
        self.variables.append(codeobj)

    def _children(self):
        """Yield all direct children of this object."""
        for codeobj in self.variables:
            yield codeobj

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        spaces = ' ' * indent
        return spaces + ', '.join(v.pretty_str() for v in self.variables)

    def __repr__(self):
        """Return a string representation of this object."""
        return str(self.variables)


class CodeControlFlow(CodeStatement, CodeStatementGroup):
    """Base class for control flow structures (e.g. `for` loops).

        Control flow statements are assumed to have, at least, one branch
        (a boolean condition and a `CodeBlock` that is executed when
        the condition is met). Specific implementations may consider
        more branches, or default branches (executed when no condition is met).

        A control flow statement typically has a name.
    """

    def __init__(self, scope, parent, name):
        """Constructor for control flow structures.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            name (str): The name of the control flow statement in the program.
        """
        CodeStatement.__init__(self, scope, parent)
        self.name = name
        self.condition = True
        self.body = CodeBlock(scope, self, explicit=False)

    def get_branches(self):
        """Return a list of branches, where each branch is a pair of
            condition and respective body."""
        return [(self.condition, self.body)]

    def _set_condition(self, condition):
        """Set the condition for this control flow structure."""
        assert isinstance(condition, CodeExpression.TYPES)
        self.condition = condition

    def _set_body(self, body):
        """Set the main body for this control flow structure."""
        assert isinstance(body, CodeStatement)
        if isinstance(body, CodeBlock):
            self.body = body
        else:
            self.body._add(body)

    def _children(self):
        """Yield all direct children of this object."""
        if isinstance(self.condition, CodeExpression):
            yield self.condition
        for codeobj in self.body._children():
            yield codeobj

    def __repr__(self):
        """Return a string representation of this object."""
        return '{} {}'.format(self.name, self.get_branches())


class CodeConditional(CodeControlFlow):
    """This class represents a conditional (`if`).

        A conditional is allowed to have a default branch (the `else` branch),
        besides its mandatory one.
    """

    def __init__(self, scope, parent):
        """Constructor for conditionals.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
        """
        CodeControlFlow.__init__(self, scope, parent, 'if')
        self.else_body = CodeBlock(scope, self, explicit=False)

    @property
    def then_branch(self):
        """The branch associated with a condition."""
        return self.condition, self.body

    @property
    def else_branch(self):
        """The default branch of the conditional."""
        return True, self.else_body

    def statement(self, i):
        """Return the *i*-th statement of this block.

            Behaves as if the *then* and *else* branches were
            concatenated, for indexing purposes.
        """
    # ----- This code is just to avoid creating a new list and
    #       returning a custom exception message.
        o = len(self.body)
        n = o + len(self.else_body)
        if i >= 0 and i < n:
            if i < o:
                return self.body.statement(i)
            return self.else_body.statement(i - o)
        elif i < 0 and i >= -n:
            if i >= o - n:
                return self.else_body.statement(i)
            return self.body.statement(i - o + n)
        raise IndexError('statement index out of range')

    def statement_after(self, i):
        """Return the statement after the *i*-th one, or `None`."""
        k = i + 1
        o = len(self.body)
        n = o + len(self.else_body)
        if k > 0:
            if k < o:
                return self.body.statement(k)
            if k > o and k < n:
                return self.else_body.statement(k)
        if k < 0:
            if k < o - n and k > -n:
                return self.body.statement(k)
            if k > o - n:
                return self.else_body.statement(k)
        return None

    def get_branches(self):
        """Return a list with the conditional branch and the default branch."""
        if self.else_branch:
            return [self.then_branch, self.else_branch]
        return [self.then_branch]

    def _add_default_branch(self, body):
        """Add a default body for this conditional (the `else` branch)."""
        assert isinstance(body, CodeStatement)
        if isinstance(body, CodeBlock):
            self.else_body = body
        else:
            self.else_body._add(body)

    def __len__(self):
        """Return the length of both branches combined."""
        return len(self.body) + len(self.else_body)

    def _children(self):
        """Yield all direct children of this object."""
        if isinstance(self.condition, CodeExpression):
            yield self.condition
        for codeobj in self.body._children():
            yield codeobj
        for codeobj in self.else_body._children():
            yield codeobj

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        spaces = ' ' * indent
        condition = pretty_str(self.condition)
        pretty = '{}if ({}):\n'.format(spaces, condition)
        pretty += self.body.pretty_str(indent=indent + 2)
        if self.else_body:
            pretty += '\n{}else:\n'.format(spaces)
            pretty += self.else_body.pretty_str(indent=indent + 2)
        return pretty


class CodeLoop(CodeControlFlow):
    """This class represents a loop (e.g. `while`, `for`).

        Some languages allow loops to define local declarations, as well
        as an increment statement.

        A loop has only a single branch, its condition plus the body
        that should be repeated while the condition holds.
    """

    def __init__(self, scope, parent, name):
        """Constructor for loops.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
            name (str): The name of the loop statement in the program.
        """
        CodeControlFlow.__init__(self, scope, parent, name)
        self.declarations = None
        self.increment = None

    def _set_declarations(self, declarations):
        """Set declarations local to this loop (e.g. `for` variables)."""
        assert isinstance(declarations, CodeStatement)
        self.declarations = declarations
        declarations.scope = self.body

    def _set_increment(self, statement):
        """Set the increment statement for this loop (e.g. in a `for`)."""
        assert isinstance(statement, CodeStatement)
        self.increment = statement
        statement.scope = self.body

    def _children(self):
        """Yield all direct children of this object."""
        if self.declarations:
            yield self.declarations
        if isinstance(self.condition, CodeExpression):
            yield self.condition
        if self.increment:
            yield self.increment
        for codeobj in self.body._children():
            yield codeobj

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        spaces = ' ' * indent
        condition = pretty_str(self.condition)
        v = self.declarations.pretty_str() if self.declarations else ''
        i = self.increment.pretty_str(indent=1) if self.increment else ''
        pretty = '{}for ({}; {}; {}):\n'.format(spaces, v, condition, i)
        pretty += self.body.pretty_str(indent=indent + 2)
        return pretty


class CodeSwitch(CodeControlFlow):
    """This class represents a switch statement.

        A switch evaluates a value (its `condition`) and then declares
        at least one branch (*cases*) that execute when the evaluated value
        is equal to the branch value. It may also have a default branch.

        Switches are often one of the most complex constructs of programming
        languages, so this implementation might be lackluster.
    """

    def __init__(self, scope, parent):
        """Constructor for switches.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
        """
        CodeControlFlow.__init__(self, scope, parent, "switch")
        self.cases = []
        self.default_case = None

    def _add_branch(self, value, statement):
        """Add a branch/case (value and statement) to this switch."""
        self.cases.append((value, statement))

    def _add_default_branch(self, statement):
        """Add a default branch to this switch."""
        self.default_case = statement

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        spaces = ' ' * indent
        condition = pretty_str(self.condition)
        pretty = '{}switch ({}):\n'.format(spaces, condition)
        pretty += self.body.pretty_str(indent=indent + 2)
        return pretty


class CodeTryBlock(CodeStatement, CodeStatementGroup):
    """This class represents a try-catch block statement.

        `try` blocks have a main body of statements, just like regular blocks.
        Multiple `catch` blocks may be defined to handle specific types of
        exceptions.
        Some languages also allow a `finally` block that is executed after
        the other blocks (either the `try` block, or a `catch` block, when
        an exception is raised and handled).
    """

    def __init__(self, scope, parent):
        """Constructor for try block structures.

        Args:
            scope (CodeEntity): The program scope where this object belongs.
            parent (CodeEntity): This object's parent in the program tree.
        """
        CodeStatement.__init__(self, scope, parent)
        self.body = CodeBlock(scope, self, explicit=True)
        self.catches = []
        self.finally_body = CodeBlock(scope, self, explicit=True)

    def _set_body(self, body):
        """Set the main body for try block structure."""
        assert isinstance(body, CodeBlock)
        self.body = body

    def _add_catch(self, catch_block):
        """Add a catch block (exception variable declaration and block)
            to this try block structure.
        """
        assert isinstance(catch_block, self.CodeCatchBlock)
        self.catches.append(catch_block)

    def _set_finally_body(self, body):
        """Set the finally body for try block structure."""
        assert isinstance(body, CodeBlock)
        self.finally_body = body

    def _children(self):
        """Yield all direct children of this object."""
        for codeobj in self.body._children():
            yield codeobj
        for catch_block in self.catches:
            for codeobj in catch_block._children():
                yield codeobj
        for codeobj in self.finally_body._children():
            yield codeobj

    def __len__(self):
        """Return the length of all blocks combined."""
        n = len(self.body) + len(self.catches) + len(self.finally_body)
        n += sum(map(len, self.catches))
        return n

    def __repr__(self):
        """Return a string representation of this object."""
        return 'try {} {} {}'.format(self.body, self.catches,
                                     self.finally_body)

    def pretty_str(self, indent=0):
        """Return a human-readable string representation of this object.

        Kwargs:
            indent (int): The amount of spaces to use as indentation.
        """
        spaces = ' ' * indent
        pretty = spaces + 'try:\n'
        pretty += self.body.pretty_str(indent=indent + 2)
        for block in self.catches:
            pretty += '\n' + block.pretty_str(indent)
        if len(self.finally_body) > 0:
            pretty += '\n{}finally:\n'.format(spaces)
            pretty += self.finally_body.pretty_str(indent=indent + 2)
        return pretty

    class CodeCatchBlock(CodeStatement, CodeStatementGroup):
        """Helper class for catch statements within a try-catch block."""

        def __init__(self, scope, parent):
            """Constructor for catch block structures."""
            CodeStatement.__init__(self, scope, parent)
            self.declarations = None
            self.body = CodeBlock(scope, self, explicit=True)

        def _set_declarations(self, declarations):
            """Set declarations local to this catch block."""
            assert isinstance(declarations, CodeStatement)
            self.declarations = declarations
            declarations.scope = self.body

        def _set_body(self, body):
            """Set the main body of the catch block."""
            assert isinstance(body, CodeBlock)
            self.body = body

        def _children(self):
            """Yield all direct children of this object."""
            if isinstance(self.declarations, CodeStatement):
                yield self.declarations
            for codeobj in self.body._children():
                yield codeobj

        def __repr__(self):
            """Return a string representation of this object."""
            return 'catch ({}) {}'.format(self.declarations, self.body)

        def pretty_str(self, indent=0):
            """Return a human-readable string representation of this object.

            Kwargs:
                indent (int): The amount of spaces to use as indentation.
            """
            spaces = ' ' * indent
            decls = ('...' if self.declarations is None
                     else self.declarations.pretty_str())
            body = self.body.pretty_str(indent=indent + 2)
            pretty = '{}catch ({}):\n{}'.format(spaces, decls, body)
            return pretty


###############################################################################
# Helpers
###############################################################################

def pretty_str(something, indent=0):
    """Return a human-readable string representation of an object.

        Uses `pretty_str` if the given value is an instance of
        `CodeEntity` and `repr` otherwise.

    Args:
        something: Some value to convert.

    Kwargs:
        indent (int): The amount of spaces to use as indentation.
    """
    if isinstance(something, CodeEntity):
        return something.pretty_str(indent=indent)
    else:
        return (' ' * indent) + repr(something)
