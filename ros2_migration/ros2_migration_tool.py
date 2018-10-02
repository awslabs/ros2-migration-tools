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


""" Command line interface for ros2_migration_tool """

import click
from porting_tools.constants import CatkinToAmentMigration
from docker_setup.constants import DEFAULT_IMAGE, DEFAULT_TAG, DEFAULT_BUILD_TAG, DEFAULT_ROS1_VERSION


@click.group()
def cli():
    pass


@cli.command()
@click.option("--src-xml", default=CatkinToAmentMigration.DEFAULT_SRC_PACKAGE_XML, help="Path to ROS1 package.xml file")
@click.option("--dst-xml", default=CatkinToAmentMigration.DEFAULT_DST_PACKAGE_XML, help="Path to write ROS2 package.xml file")
@click.option("--src-cmake", default=CatkinToAmentMigration.DEFAULT_SRC_CMAKE_LISTS, help="Path to ROS1 CMakeLists.txt file")
@click.option("--dst-cmake", default=CatkinToAmentMigration.DEFAULT_DST_CMAKE_LISTS, help="Path to write ROS2 CMakeLists.txt file")
@click.option("--package-path", default=None, help="If provided, this path will override manually set source/destination files and will overwrite the CMakeLists.txt and package.xml files in the given package.")
def catkin_to_ament(src_xml, dst_xml, src_cmake, dst_cmake, package_path):
    """ Ports CMakeLists and package.xml from catkin to ament """
    import porting_tools.catkin_to_ament
    porting_tools.catkin_to_ament.catkin_to_ament(src_xml=src_xml,
                                                  dst_xml=dst_xml,
                                                  src_cmake=src_cmake,
                                                  dst_cmake=dst_cmake,
                                                  package_path=package_path)


@cli.command()
@click.option("--src", help="Path to ros1 source cpp file")
@click.option("--dst", help="Path to ros2 output file")
def port_cpp(src, dst):
    """ Attempts to port ROS1 c++ source code to ROS2 """
    import porting_tools.cpp_source_porter
    porting_tools.cpp_source_porter.port_cpp(src=src,
                                             dst=dst)


@cli.command()
@click.option("--src", help="Path to ros1 source python file")
@click.option("--dst", help="Path to ros2 output file")
def port_python(src, dst):
    """ Attempts to port ROS1 python source code to ROS2 """
    import porting_tools.python_source_porter
    porting_tools.python_source_porter.port_python(src=src,
                                                   dst=dst)

@cli.command()
@click.option("--image", default=DEFAULT_IMAGE, help="The docker image to pull")
@click.option("--tag", default=DEFAULT_TAG, help="Optional tag for image")
@click.option("--src-volume", default=None, help="Directory to mount")
@click.option("--dst-dir", default=None, help="Destination in container to mount")
@click.option("--build-tag", default=DEFAULT_BUILD_TAG, help="Tag for the built image")
@click.option("--ros1-version", default=DEFAULT_ROS1_VERSION, help="ROS1 version to install in the container")
def launch_docker(image, tag, src_volume, dst_dir, build_tag, ros1_version):
    """ Launches a docker container setup for ROS1 to ROS2 migration """
    import docker_setup.docker_setup as ds
    ds.docker_setup(image_id=image,
                    tag=tag,
                    src_volume=src_volume,
                    dst_dir=dst_dir,
                    build_tag=build_tag,
                    ros1_version=ros1_version)


@cli.command()
@click.argument("pkg")
@click.option("--status", default=None, help="new status of package")
@click.option("--detail", default=None, help="new details of package")
def update_pkg(pkg, status, detail):
    """ Updates the status and details of a package in the Unofficial yaml list"""
    from effort_estimation import ros2_release_checker
    ros2_release_checker.update_unofficial_status(pkg=pkg,
                                                  status=status,
                                                  detail=detail)

@cli.command()
@click.argument("file_name")
def check_source(file_name):
    """ Counts instances of ros:: or rospy in a file to estimate effort of migration """
    from effort_estimation import effort_estimation
    effort_estimation.check_source(file_name)

@cli.command()
@click.argument("package")
def check_deps(package):
    """ Checks the migration status of the dependencies of a package """
    from effort_estimation import effort_estimation
    effort_estimation.check_dependencies(package)


if __name__ == '__main__':
    cli()
