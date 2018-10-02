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

import csv
import sys
import pickle
import utils
import argparse

DATA_NAME_IDX = 0
DATA_ROS_USAGE_IDX = 1
DATA_DEP_COUNT_IDX = 2
DATA_ALL_DEP_COUNT_IDX = 3
DATA_MAINTAINER_IDX = 4

CSV_NAME_IDX = 0
CSV_EFFORT_IDX = 3
CSV_MAINTAINER_IDX = 9

def _is_size_large(item):
    return True if (item[DATA_ROS_USAGE_IDX] >= 30 or item[DATA_DEP_COUNT_IDX] >= 3) else False

def _is_size_medium(item):
    return True if (3 > item[DATA_DEP_COUNT_IDX] > 0 and item[DATA_ROS_USAGE_IDX] < 30) else False

def _is_size_small(item):
    return item[DATA_DEP_COUNT_IDX] == 0 and not _is_size_medium(item) and not _is_size_large(item)

def categorize_packages(data):
    sizes = {}
    for item in data:
        if type(item[DATA_DEP_COUNT_IDX]) is list:
            item = list(item)
            item[DATA_DEP_COUNT_IDX] = len(item[DATA_DEP_COUNT_IDX])
        if _is_size_small(item):
            sizes[item[DATA_NAME_IDX]] = 'S'
        elif _is_size_medium(item):
            sizes[item[DATA_NAME_IDX]] = 'M'
        else:
            sizes[item[DATA_NAME_IDX]] = 'L'
    return sizes

def get_maintainers(data):
    maintainers = {}
    for item in data:
        if item[DATA_MAINTAINER_IDX]:
            maintainers[item[DATA_NAME_IDX]] = ','.join(item[DATA_MAINTAINER_IDX])
    return maintainers

def _get_info_for_csv_row(row, tshirt_sizes, maintainers):
    pkg_name = row[CSV_NAME_IDX].strip()
    tshirt_size = None
    maintainer = None
    for name in utils.generate_ros_package_names(pkg_name):
        if name in tshirt_sizes:
            tshirt_size = tshirt_sizes[name]
        if name in maintainers:
            maintainer = maintainers[name]
    return tshirt_size, maintainer

def update_csv(csv_filename, tshirt_sizes, maintainers):
    csv_data = []
    with open(csv_filename, 'rb') as csv_file:
        reader = csv.reader(csv_file)
        for row in reader:
            if not row or not row[0]:
                continue
            tshirt_size, maintainer = _get_info_for_csv_row(row, tshirt_sizes, maintainers)
            if CSV_EFFORT_IDX and tshirt_size:
                row[CSV_EFFORT_IDX] = tshirt_size
            if CSV_MAINTAINER_IDX and maintainer:
                row[CSV_MAINTAINER_IDX] = maintainer
            csv_data.append(row)

    with open(csv_filename, 'wb') as csv_file:
        writer = csv.writer(csv_file)
        for row in csv_data:
            writer.writerow(row)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data_filename", help="a pickled list of tuples")
    parser.add_argument("csv_filename", help="a CSV file with package names at column %d and effort estimation at column %d (configurable)" % (CSV_NAME_IDX, CSV_EFFORT_IDX, ))
    parser.add_argument("--maintainer-only", action="store_true", help="Skip effort estimation - only update the maintainers")

    args = parser.parse_args()
    with open(args.data_filename, 'rb') as data_file:
        data = pickle.load(data_file)

    tshirt_sizes = {}
    if not args.maintainer_only:
        tshirt_sizes = categorize_packages(data)

    maintainers = get_maintainers(data)
    update_csv(args.csv_filename, tshirt_sizes, maintainers)

if __name__ == '__main__':
    main()
