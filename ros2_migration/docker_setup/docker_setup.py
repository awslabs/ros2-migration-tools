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

"""
Tool to pull ros2 migration docker image

Usage: python3 docker_setup.py -h

"""


import os
import argparse
import subprocess
try:
    import docker
except ImportError as error:
    print(error)
    print("Install the necessary packages or pip install . to install all missing packages")
    exit(1)
from .constants import DEFAULT_IMAGE, DEFAULT_TAG, DEFAULT_BUILD_TAG, DEFAULT_ROS1_VERSION


DOCKER_SETUP_PATH = os.path.dirname(os.path.abspath(__file__)) #path to this file
DOCKERFILE_TEMPLATE = os.path.join(DOCKER_SETUP_PATH, "assets", "Dockerfile_template")
DOCKERFILE_OUPTUT = os.path.join(DOCKER_SETUP_PATH, "assets", "Dockerfile")



def get_arg_parser():
    parser = argparse.ArgumentParser(description='Pull a docker image and setup workspace for ros2 migration')
    parser.add_argument('--image', default=DEFAULT_IMAGE, type=str, help="The docker image to pull")
    parser.add_argument('--tag', default=DEFAULT_TAG, type=str, help="Optional tag for image")
    parser.add_argument('--src-volume', default=None, type=str, help="Directory to mount")
    parser.add_argument('--dst-dir', default=None, type=str, help="Destination in container to mount")
    parser.add_argument('--build-tag', default=DEFAULT_BUILD_TAG, type=str, help="tag for the built image")
    parser.add_argument('--ros1_version', default=DEFAULT_ROS1_VERSION, type=str, help="ROS1 version to install in the container")
    return parser


def validate_args(image, tag, src_volume, dst_dir):
    """Checks that the src_volume and dst_dir are valid"""
    if (src_volume is None) is not (dst_dir is None):
        print("Warning: both a src and destination volume must be specificed for the volume to be mounted")
    if src_volume is not None and not os.path.exists(os.path.abspath(src_volume)):
        raise FileNotFoundError(src_volume)


def write_dockerfile(image, tag, ros1_version):
    with open(DOCKERFILE_TEMPLATE, 'r') as template_file:
        template = template_file.read()
    output = template.format(image=image, tag=tag, ros1_version=ros1_version)
    with open(DOCKERFILE_OUPTUT, 'w') as dockerfile:
        dockerfile.write(output)


def docker_setup(image_id, tag, build_tag, src_volume, dst_dir, ros1_version):
    validate_args(image=image_id,
                  tag=tag,
                  src_volume=src_volume,
                  dst_dir=dst_dir)

    docker_client = docker.from_env()
    print("Writing Dockerfile")
    write_dockerfile(image=image_id, tag=tag, ros1_version=ros1_version)
    print("Building image")
    (image, logs) = docker_client.images.build(path=os.path.dirname(DOCKERFILE_OUPTUT), tag=build_tag)
    for log in logs:
        print(log)
    print("Image built")

    #If either a source volume or destination directory have not been specified then the volume cannot be mounted
    if not src_volume or not dst_dir:
        mount_command = ""
    #Otherwise mount the volume
    else:
        mount_command = " --mount type=bind,src={},dst={} ".format(os.path.abspath(src_volume), dst_dir)

    print("Running docker container")

    run_command = "docker run -it {image}"
    subprocess.run("docker run -it {mount_command} {image}".format(image=image.id, mount_command=mount_command).split())
    print("Finishing")


def main():
    parser = get_arg_parser()
    args = parser.parse_args()
    docker_setup(image_id=args.image,
                 tag=args.tag,
                 src_volume=args.src_volume,
                 dst_dir=args.dst_dir,
                 build_tag=args.build_tag,
                 ros1_version=args.ros1_version)



if __name__ == '__main__':
    main()
