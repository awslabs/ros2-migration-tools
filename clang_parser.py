# This file is part of libigl, a simple c++ geometry processing library.
#
# Copyright (C) 2017 Sebastian Koch <s.koch@tu-berlin.de> and Daniele Panozzo <daniele.panozzo@gmail.com>
#
# This Source Code Form is subject to the terms of the Mozilla Public License
# v. 2.0. If a copy of the MPL was not distributed with this file, You can
# obtain one at http://mozilla.org/MPL/2.0/.

from clang import cindex as clang
from pprint import pprint


def traverse(c, path, objects):

    if c.location.file and not c.location.file.name.endswith(path):
        return

    if c.spelling == "PARULA_COLOR_MAP":  # Fix to prevent python stack overflow from infinite recursion
        return

    text = c.spelling or c.displayname
    kind = str(c.kind)[str(c.kind).index('.') + 1:]

    identifier = text + ":-> " + kind

    # print(identifier)

    if c.kind == clang.cindex.CursorKind.TRANSLATION_UNIT or c.kind == clang.cindex.CursorKind.UNEXPOSED_DECL:
        # Ignore  other cursor kinds
        pass

    elif c.kind == clang.cindex.CursorKind.NAMESPACE:
        objects["namespaces"].append(identifier)
        print("Namespace", c.spelling, c.get_children())
        pass

    elif c.kind == clang.cindex.CursorKind.FUNCTION_TEMPLATE:
        print("Function Template", c.spelling, c.raw_comment)
        objects["functions"].append(identifier)
        return

    elif c.kind == clang.cindex.CursorKind.FUNCTION_DECL:
        print("FUNCTION_DECL", c.spelling, c.raw_comment)
        objects["functions"].append(identifier)
        return

    elif c.kind == clang.cindex.CursorKind.ENUM_DECL:
        print("ENUM_DECL", c.spelling, c.raw_comment)
        objects["enums"].append(identifier)
        return

    elif c.kind == clang.cindex.CursorKind.CLASS_DECL:
        objects["classes"].append(identifier)
        return

    elif c.kind == clang.cindex.CursorKind.CLASS_TEMPLATE:
        objects["classes"].append(identifier)
        return

    elif c.kind == clang.cindex.CursorKind.STRUCT_DECL:
        objects["structs"].append(identifier)
        return
    elif c.kind == clang.cindex.CursorKind.CALL_EXPR:
        objects["call_expr"].append(identifier)
        return

    else:
        # print("Unknown", c.kind, c.spelling)
        pass

    for child_node in c.get_children():
        traverse(child_node, path, objects)


def parse(path):
    clang.Config.set_library_path('./clang')

    index = clang.cindex.Index.create()
    args = ['-x', 'c++', '-std=c++14', '-fparse-all-comments', '-DIGL_STATIC_LIBRARY']
    tu = index.parse(path, args)

    objects = {"functions": [], "enums": [], "namespaces": [], "classes": [], "structs": [], "call_expr": []}
    traverse(tu.cursor, path, objects)

    return objects


if __name__ == '__main__':
    pprint(parse("ros1_talker_small.cpp"))
