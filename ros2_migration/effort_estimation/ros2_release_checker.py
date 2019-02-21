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


""" Manages checking the release status of ros2 packages """
import subprocess
import os
import yaml



DISTRIBUTION_YAML_URLS = ['https://raw.githubusercontent.com/ros2/rosdistro/ros2/ardent/distribution.yaml',
                          'https://raw.githubusercontent.com/ros2/rosdistro/ros2/bouncy/distribution.yaml', ]


IGNORE_PACKAGES = ['roscpp', 'rospy', 'roscpp_traits', 'roscpp_serialization', 'catkin',
                   'message_generation', 'message_runtime', 'gencpp', 'genpy', 'genmsg',
                   'genlisp', 'rosmaster', 'rosbuild', 'nodelet', 'geneus', 'gennodejs',
                   'rosmsg', 'rosparam', 'rostopic', 'rosservice', 'xmlrpcpp', 'cpp_common',
                   'rosgraph_msgs']


class MigrationStatus():
    """ Holds the migration status of a ros2 package """
    released = "Released"
    ignore = "Ignore"
    not_migrated = "Not Migrated"
    unknown = "Unknown Status"
    def __init__(self, status=None, detail=""):
        self.status = MigrationStatus.unknown if status is None else status
        self.detail = detail

    def __str__(self):
        if not self.detail:
            return self.status
        return "%s (%s)" % (self.status, self.detail)

    def to_dict(self):
        return {"status":self.status, "detail":self.detail}


DIR_PATH = os.path.dirname(os.path.abspath(__file__)) #path to this file
UNOFFICIAL_YAML_FILE = os.path.join(DIR_PATH, "unofficial_migration_status.yaml")


def get_ros2_distros():
    """ Pulls distrobutions from ros2 github """
    distrib_yamls = []
    for url in DISTRIBUTION_YAML_URLS:
        distrib_yamls.append(yaml.load(subprocess.check_output('curl %s' % (url, ), shell=True)))
    return distrib_yamls

def get_ros2_release():
    """
    Returns dictionary of release status from ros2 distros
    """
    distrib_dict = {}

    for distrib_yaml in get_ros2_distros():
        for _, info in distrib_yaml["repositories"].items():
            if "release" in info and "packages" in info["release"]:
                for pkg in info["release"]["packages"]:
                    distrib_dict[pkg] = MigrationStatus(status=MigrationStatus.released, detail=info["release"]["url"])

    return distrib_dict


def get_unofficial_status():
    packages = {}
    with open(UNOFFICIAL_YAML_FILE, 'r') as file:
        unoffical = yaml.load(file.read())
    for pkg, info in unoffical["packages"].items():
        packages[pkg] = MigrationStatus(status=info.get("status", ""), detail=info.get("detail", ""))
    return packages


def get_ignore_packages():
    return {pkg:MigrationStatus(status=MigrationStatus.ignore) for pkg in IGNORE_PACKAGES}


def update_unofficial_status(pkg, status=None, detail=None):
    """ Updates the unoficial status in the UNOFFICIAL_YAML_FILE """
    statuses = get_unofficial_status()

    #If package already has a status or details and no new one is
    #provided, leave them the same
    if pkg in statuses:
        if status is None:
            status = pkg.status
        if detail is None:
            detail = pkg.detail
    #If package isn't in statuses and no new status or detail is provided
    #just put in blank strings
    else:
        if status is None:
            status = ""
        if detail is None:
            detail = ""

    statuses[pkg] = MigrationStatus(status=status, detail=detail)
    with open(UNOFFICIAL_YAML_FILE, 'w') as file:
        #convert MigrationStatus objects back to dicts before writing
        yaml.dump([pkg.to_dict() for pkg in statuses], file)


def get_package_status(pkgs):
    """
    Takes a list of package names
    returns a dictionary of the form {<pkg name> : <release status string>}
    """

    #combine packages from official status, unofficial, and ignored
    packages = get_unofficial_status()
    packages.update(get_ros2_release())
    packages.update(get_ignore_packages())

    statuses = {}
    for pkg in pkgs:
        statuses[pkg] = packages.get(pkg, MigrationStatus.not_migrated)

    return statuses
