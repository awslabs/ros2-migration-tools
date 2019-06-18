#!/usr/bin/env python
# -*- coding: utf-8 -*-

from argparse import ArgumentParser, FileType
from clang import cindex
import os, sys
def node_info(node):
    return {'kind': node.kind,
            'usr': node.get_usr(),
            'spelling': node.spelling,
            'location': node.location,
            'file': node.location.file.name,
            'extent.start': node.extent.start,
            'extent.end': node.extent.end,
            'is_definition': node.is_definition()
            }


def get_nodes_in_file(node, filename, ls=None):
    ls = ls if ls is not None else []
    for n in node.get_children():
        if n.location.file is not None and n.location.file.name == filename:
            ls.append(n)
            get_nodes_in_file(n, filename, ls)
    return ls


def main():
    cindex.Config.set_library_path('./clang')
    arg_parser = ArgumentParser()
    arg_parser.add_argument('source_file', type=FileType('r+'),
                            help='C++ source file to parse.')
    arg_parser.add_argument('compilation_database', type=FileType('r+'),
                            help='The compile_commands.json to use to parse the source file.')
    args = arg_parser.parse_args()

    compilation_database_path = args.compilation_database.name
    source_file_path = args.source_file.name
    index = cindex.Index.create()

    compdb = cindex.CompilationDatabase.fromDirectory(os.path.dirname(compilation_database_path))
    file_args = compdb.getCompileCommands(source_file_path)

    clang_args = ['-x', 'c++', '-std=c++11', '-p', compilation_database_path]
    argg = ['-x', 'c++', '-std=c++11', '-D__CODE_GENERATOR__']
    # file_args = "/usr/bin/c++ -std=gnu++14 -o CMakeFiles/example.dir/example.cpp.o".split(' ')

    # cwd = os.getcwd()
    # os.chdir('sample')
    translation_unit = index.parse(source_file_path, argg)
    # os.chdir(cwd)

    file_nodes = get_nodes_in_file(translation_unit.cursor, source_file_path)
    print [p.spelling for p in file_nodes]


if __name__ == '__main__':
    main()
