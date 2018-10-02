Tools in this subpackage:

1. ROS2 migration effort estimator
        usage: effort_estimation.py <packages_file>
          packages_file:  Path to a file containing newline-separated list of package names

2. Categorize and dump to CSV
        usage: effort_estimation_to_csv.py <data file> <csv to update>
          data file: a pickled list of tuples containing package name, ROS usage count, and ROS dependency count.
          csv to update: a CSV file with package names at column 0 and effort estimation at column 3 (configurable).

3. ROS2 release checker - given a list of package names, check which ones have been released to ROS2
        usage: python -i ros2_release_checker.py
          example:
            `>>> get_released_packages(['image_geometry', 'opencv3'])`
            `['image_geometry']`
