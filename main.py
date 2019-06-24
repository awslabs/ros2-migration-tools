from ros_upgrader import RosUpgrader
import json


def main():
    RosUpgrader.init()

    RosUpgrader.init_ros2_folder()
    RosUpgrader.convert_all_cmake()
    RosUpgrader.convert_all_package_xml()

    # print(json.dumps(RosUpgrader.get_tokens_as_json(), indent=4))

if __name__ == '__main__':
    main()