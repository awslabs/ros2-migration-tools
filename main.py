from ros_upgrader import RosUpgrader
import json


def main():
    RosUpgrader.init()
    print(json.dumps(RosUpgrader.get_tokens_as_json(), indent=4))


if __name__ == '__main__':
    main()