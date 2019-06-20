from bonsai.cpp.clang_parser import CppAstParser
import os

def get_abs_path(rel_path):
    return os.path.abspath(rel_path)

def main():
    CppAstParser.set_library_path(get_abs_path("./clang"))
    CppAstParser.set_database(get_abs_path("./sample"))

    work_space = "sample"
    user_includes = [get_abs_path("./sample")]
    cp = CppAstParser(work_space, user_includes)
    print(cp.get_ast("sample/example.cpp"))

    #print(cp.parse("sample/example.cpp"))


if __name__ == '__main__':
    main()
