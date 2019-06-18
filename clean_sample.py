import os, shutil, sys

def clean_sample_folder(folder):
    try:
        os.remove(os.path.join(folder, "cmake_install.cmake"))
        os.remove(os.path.join(folder, "CMakeCache.txt"))
        os.remove(os.path.join(folder, "compile_commands.json"))
        os.remove(os.path.join(folder, "Makefile"))
        shutil.rmtree(os.path.join(folder, "CMakeFiles"))
    except OSError as e:  ## if failed, report it back to the user ##
        print ("Error: %s - %s." % (e.filename, e.strerror))


clean_sample_folder(".")