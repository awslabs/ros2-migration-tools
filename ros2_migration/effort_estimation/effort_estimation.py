# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#  
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#  
#      http://www.apache.org/licenses/LICENSE-2.0
#  
# or in the "license" file accompanying this file. This file is distributed 
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either 
# express or implied. See the License for the specific language governing 
# permissions and limitations under the License.


"""Script for estimating time to migrate a package and check dependencies """
import argparse
import subprocess
import os
import pickle
from .ros2_release_checker import get_package_status


ROS_USAGE_GREP = 'grep -nr "ros::|rospy." %s '

def fetch_repo(pkg, distro):
    """
    Pulls a ros repo
    Arguments:
        pkg - the name of the package to check
        distro - the distribution to check
    """
    url = subprocess.check_output('roslocate uri %s --rel --distro %s' % (pkg, distro, ), shell=True).strip()
    os.system('rm -rf src build devel 2>/dev/null')
    os.system('mkdir src')
    clone_cmd = 'git clone %s src 2>/dev/null' % (url, )
    subprocess.check_output(clone_cmd, shell=True)


def get_dependencies(pkg):
    """
    Returns the dependencies of a package
    Arguments:
        pkg - the name of the package to check
    Returns:
        direct_dependencies - a list of names of direct dependencies of pkg
        dependencies - a list of all dependencies of pkg (recursively checks dependencies)
    """
    try:
        subprocess.check_output('rosdep install --from-paths src --ignore-src -r -y', shell=True)
    except Exception as e:
        print('Tried rosdep install but failed, continuing as a best effort.', e)
    subprocess.check_output('catkin_make -DCMAKE_BUILD_TYPE=Release', shell=True)
    direct_dependencies = subprocess.check_output('/bin/bash -c "source devel/setup.bash && rospack depends1 %s"' % (pkg, ), shell=True).strip().decode("utf-8").split()
    dependencies = subprocess.check_output('/bin/bash -c "source devel/setup.bash && rospack depends %s"' % (pkg, ), shell=True).strip().decode("utf-8").split()
    os.system('rm -rf build devel 2>/dev/null')
    return direct_dependencies, dependencies

def check_source(path):
    """Prints all usage of "ros" in a file """
    print(subprocess.check_output(ROS_USAGE_GREP % (path, ), shell=True).decode('ascii'))

def get_ros_usage_count(path):
    """
    Counts the number of times ros is used in a package to estimate effort
    Arguments:
        path - path to the file to check
    Returns:
        number of lines containing the string "ros:: or rospy
    """
    lines_of_code = int(subprocess.check_output(ROS_USAGE_GREP % (path, ) + ' | wc -l', shell=True).strip())
    return lines_of_code

def get_maintainers(path):
    """
    Finds the maintainers of a package
    Arguments:
        path - path to directory containing package.xml
    Returns:
        list of maintainer names and emails
    """
    package_xml = os.path.join(path, 'package.xml')
    if not os.path.isfile(package_xml):
        return None
    else:
        maintainers = subprocess.check_output(r'grep -Po "maintainer email=\".*?\">" %s | grep -Po "\".*?\""' % (package_xml, ), shell=True).strip()
        if maintainers:
            return [maintainer.strip().replace('"', '') for maintainer in maintainers.split('\n') if maintainer.strip()]

def analyze_code(pkg, no_compile):
    """
    Checks the ros usage and maintainers fo a package
    Arguments:
        pkg - the name of the package to check
        no_compile - option to not compile the ROS package
    Returns:
        - the ros usage count
        - the list of maintainers
    """

    if not no_compile:
        # catkin_make was invoked; get path via rospack find
        path = subprocess.check_output('rospack find %s' % (pkg, ), shell=True).strip()
    else:
        # catkin_make wasn't invoked; use catkin tools instead
        subprocess.check_output('catkin init', shell=True)
        path = subprocess.check_output('catkin locate %s' % (pkg, ), shell=True).strip()
    return get_ros_usage_count(path), get_maintainers(path)

def check_dependencies(pkg):
    """
    Prints a list of dependencies and their migration statuses
    Arguments:
        pkg - the package to check dependencies of
    """
    _, all_deps = get_dependencies(pkg)

    for package, status in get_package_status(pkgs=all_deps).items():
        print("%s - %s" %(package, status))

def estimate_effort(packages, no_compile):
    stats = []
    success_logfile = 'data.txt'
    success_picklefile = 'data'
    error_logfile = 'errors.txt'
    print('Writing textual data to "%s", pickled data to "%s", errors to "%s"' % (success_logfile, success_picklefile, error_logfile, ))
    print('* Hint: Data is in the form <package name, ROS line count, direct dependencies, all dependencies, maintainers>')
    os.system('rm %s %s 2>/dev/null' % (success_logfile, error_logfile, ))
    for package_name in packages:
        should_get_line_count_for_ws = False
        try:
            print('>>>>> Processing', package_name)
            indigo_fallback = False
            try:
                fetch_repo(package_name, 'kinetic')
            except Exception as e:
                print('Caught %s trying to fetch repo in kinetic, trying indigo' % (e, ))
                fetch_repo(package_name, 'indigo')
                indigo_fallback = True
            should_get_line_count_for_ws = True # Fetched repo successfully, so if catkin_make fails we can still grep
            if not no_compile:
                direct_deps, all_deps = get_dependencies(package_name)
            else:
                direct_deps, all_deps = [], []

            migrated_packages = get_package_status(pkgs=all_deps)
            lines, maintainers = analyze_code(package_name, no_compile)

            print('>>>>>>>> %d lines used ROS API, %d direct dependencies, %d total dependencies, maintainers: %s' % (lines, len(direct_deps), len(all_deps), maintainers, ))
            stats.append((package_name, lines, direct_deps, all_deps, maintainers))
            with open(success_logfile, 'a') as f:
                f.write('%s, %d, %s, %s, %s\n' % (package_name, lines, direct_deps, all_deps, maintainers, ))
                for dependency in all_deps:
                    migration_status = migrated_packages[dependency]
                    f.write('\t%-25s - %s\n' % (dependency, migration_status))
        except Exception as e:
            print('Unexpected error', e)
            with open(error_logfile, 'a') as f:
                f.write('%s,%s\n' % (package_name, e))
            if should_get_line_count_for_ws:
                lines = get_ros_usage_count('src')
                with open(success_logfile, 'a') as f:
                    f.write('%s, %d, Unknown, Unknown\n' % (package_name, lines, ))
            continue

    with open(success_picklefile, 'wb') as output:
        pickle.dump(stats, output)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("packages_file", help="Path to a file containing newline-separated list of package names")
    parser.add_argument("--no-compile", action="store_true", help="Skip catkin_make, do not process dependencies (outputs only line count and maintainers)")
    args = parser.parse_args()
    with open(args.packages_file, 'r') as f:
        lines = f.readlines()
        pkgs = [pkg.strip() for pkg in lines]
    estimate_effort(pkgs, args.no_compile)



if __name__ == '__main__':
    main()
