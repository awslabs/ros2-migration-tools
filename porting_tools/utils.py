# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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


class MigrationWarning():
    def __init__(self, line_number, line, reason):
        self.line_number = line_number
        self.line = line
        self.reason = reason

    def __str__(self):
        return "Warning: %s\n Line number %s:\n %s\n" % (self.reason, str(self.line_number), self.line)


def find_warnings(source_lines, warning_condition, warning_msg):
    warnings = []
    for line_number, line in enumerate(source_lines):
        if warning_condition(line):
            warnings.append(MigrationWarning(line_number+1, line, warning_msg(line)))
    return warnings


def get_functions_with(criteria, from_class):
    # returns a list of functions from a class with names that meet the given criteria
    # Only take objects that have a "__func__" attribute to avoid taking fields
    functions = [(name, func) for (name, func) in from_class.__dict__.items() if hasattr(func, "__func__")]
    return [pair[1].__func__ for pair in filter(lambda pair: criteria(pair[0]), functions)]
