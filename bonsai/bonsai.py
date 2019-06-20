
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

import argparse
import cPickle
import importlib
import logging
import os


###############################################################################
# Globals
###############################################################################

_log = logging.getLogger()


###############################################################################
# Functions
###############################################################################

def parse_arguments(argv, source_runner):
    parser = argparse.ArgumentParser(prog="bonsai",
            description="Small and sane parsing trees.")
    parser.add_argument("--debug", action = "store_true",
                        help = "set debug logging")
    parser.add_argument("-f", "--format", default = "pickle",
                        choices = ["text", "bonsai", "ast", "pickle"],
                        help = "set the output format")
    parser.add_argument("-o", "--output", help = "file to store output")
    subparsers = parser.add_subparsers()

    parser_cpp = subparsers.add_parser("cpp")
    parser_cpp.add_argument("-c", "--compiler", default="clang",
                            help="parsing library (default: clang)")
    parser_cpp.add_argument("-l", "--lib-path",
                            help = "library path (required for clang)")
    parser_cpp.add_argument("-s", "--std-includes",
                            help = "standard include path")
    parser_cpp.add_argument("-w", "--workspace",
                            default = os.path.expanduser("~"),
                            help = "source workspace (default: user home)")
    parser_cpp.add_argument("-d", "--compile-db",
                            help = "compilation database directory")
    parser_cpp.add_argument("files", nargs = "+", help = "files to parse")
    parser_cpp.set_defaults(parser = parse_cpp, source_runner = source_runner)

    return parser.parse_args() if argv is None else parser.parse_args(argv)


def parse_cpp(args):
    parmod = importlib.import_module("..cpp." + args.compiler + "_parser",
                                     package = __name__)
    if args.lib_path:
        parmod.CppAstParser.set_library_path(args.lib_path)
    else:
        parmod.CppAstParser.set_library_path()
    if args.compile_db:
        parmod.CppAstParser.set_database(args.compile_db)
    parser = parmod.CppAstParser(workspace = args.workspace)
    if args.format == "ast":
        output = []
        for f in args.files:
            output.append("# " + f)
            output.append(parser.get_ast(os.path.abspath(f)))
        return "\n".join(output)
    else:
        for f in args.files:
            if parser.parse(os.path.abspath(f)) is None:
                raise ValueError("no compile commands for file " + f)
    return parser


def bonsai_format(codeobj):
    depth = 0
    par = codeobj.parent
    while not par is None:
        depth += 1
        par = par.parent
    lines = []
    for obj in codeobj.walk_preorder():
        i = 0
        par = obj.parent
        while not par is None:
            i += 1
            par = par.parent
        lines.append(obj.ast_str(indent = max(0, i - depth)))
    return "\n".join(lines)


def main(argv = None, source_runner = False):
    args = parse_arguments(argv, source_runner)
    if args.debug:
        logging.basicConfig(filename = "bonsai_log.txt", filemode = "w",
                            level = logging.DEBUG)
    else:
        logging.basicConfig(level = logging.WARNING)
    try:
        _log.info("Executing selected parser.")
        parser = args.parser(args)
        if args.format == "ast":
            text = parser
        elif args.format == "bonsai":
            text = bonsai_format(parser.global_scope)
        else:
            text = parser.global_scope.pretty_str()
        print text
        if args.output:
            _log.debug("Saving output to %s", args.output)
            with open(args.output, "w") as handle:
                if args.format == "pickle":
                    cPickle.dump(parser, handle, cPickle.HIGHEST_PROTOCOL)
                else:
                    handle.write(text)
        return 0
    except RuntimeError as err:
        _log.error(str(err))
        return 1
