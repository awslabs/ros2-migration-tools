# ROS2 Migration Tools
This repo contains a set of tools for migrating a ROS1 package to a ROS2 package.
The C++ source code migration uses [libclang8](http://releases.llvm.org/download.html) and its corresponding [python bindings](https://github.com/llvm-mirror/clang/tree/release_80/bindings/python).
The main script for migration is `ros_upgrader.py`


## Components of a ROS Package

### `CMakeLists.txt` and `package.xml`
This set of tools supports the migration of `CMakeLists.txt` and `package.xml` files.

### Source Code
This set of tools supports the migration of C++ source codes.

### msg/srv files
This set of tools does not handle any changes to message or service declarations.

### Launch files
This set of tools currently does not support migrating ROS1 launch files to ROS2 launch files.


## Pre-requisites

1. ROS1 system
2. Python-3.5 or higher
3. parse-cmake: Install using `pip3 install parse_cmake`


## Setup the Migration Tool

1. On the ROS1 system, clone the `ROS2 Migration Tools` repository:

    `git clone https://github.com/awslabs/ros2-migration-tools.git`

2. Download the `LLVM 8.0.0` `Pre-Built Binaries` for your version of Linux from [llvm download page](http://releases.llvm.org/download.html).
For example, download [Ubuntu 16.04 (.sig)](http://releases.llvm.org/8.0.0/clang+llvm-8.0.0-x86_64-linux-gnu-ubuntu-16.04.tar.xz)
if your machine is running Ubuntu 16.06.

3. Extract the downloaded tarball (for example, `tar xvf clang+llvm-8.0.0-x86_64-linux-gnu-ubuntu-16.04.tar.xz`).

4. Copy the `libclang.so.8` shared object and the `libclang.so` symlink from `lib` folder of clang to the `ros2-migration-tools/clang` folder.
    ```
    cp -r <extracted directory>/lib/libclang.so <ros2-migration-tools directory>/clang
    cp -r <extracted directory>/lib/libclang.so.8 <ros2-migration-tools directory>/clang
    ```

5. Copy the contents of the libclang `include` folder to the `ros2-migration-tools/clang/clang` folder.
    ```
    cp -r <extracted directory>/lib/clang/8.0.0/include <ros2-migration-tools directory>/clang/clang
    ```


## Setup the ROS1 Packages

1. Clone the sources of the ROS1 package which you want to port. Lets say the package cloned is named `ROS1_Package`.
`ros2-migration-tools/ros_upgrader.py` will need path to `package.xml` which can be provided either using environment variable `ROS1_PACKAGE_PATH`
or using the  `--package_xml_path` argument of `ros2-migration-tools/ros_upgrader.py`

3. Build the `ROS1_Package` ROS1 package with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

    `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

5. Building the `ROS1_Package` with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` will have created a `compile_commands.json` file inside `build/ROS1_Package`
if the build was successful. This path will be mandatory argument for `ros2-migration-tools/ros_upgrader.py` script.


## Usage

1. `cd` to `ros2-migration-tools`.

2. run `ros_upgrader.py` script with following arguments.
    - `-c` or `--compile_db_path` (required): this is path to the `compile_commands.json` file
    - `-p` or `--package_xml_path` (optional): this is path to the `package.xml` file
    - `-m` or `--mapping_file` (optional): this is name of the file to which filled mappings will be written to
    - `-f` or `--filter_out_file` (optional): this is name of the file to which unfilled tokens will be written to
    - `-o` or `--output_folder` (optional): output directory where the ported package will copied
    - `-d` or `--debug` (optional): add this flag if you want to dump the Abstract Syntax Tree created

    For example,

    ```
    export ROS1_PACKAGE_PATH=ROS1_Package/package.xml
    ros_upgrader.py -c ROS1_Package/build/ROS1_Package/compile_commands.json
    ```

    or

    ```
    ros_upgrader.py -c ROS1_Package/build/ROS1_Package/compile_commands.json -p ROS1_Package/package.xml
    ```

*Note: Paths can also be relative to the `ros_upgrader.py` package. `ROS1_Package/package.xml` must be provided by one of the two ways explained.*

3. One of the following can happen now:
    - If there are no new tokens encountered in the ROS1 package, then it will just finish running
    - If there are some new tokens encountered, there will be message on terminal saying
    `Open mapping/new_tokens.json and fill the mappings. Press 'Y' to continue or any other key to abort`
    will be shown on terminal. **Don't press 'Y' yet.** See the [Filling the mappings](##filling-the-mappings) section
    on for more details about the mappings

4. Default output folder is a folder named `output` inside the `ros2-migration-tools` or the output folder will be the folder which you specified with `-o` or `--output_folder` argument.
The default folder will have a folder with unique name *`currData_currTime`* (e.g. `2019-07-15_20_30_55`). This output folder will contain the package migrated to ROS2


## Filling the mappings

- Open `mapping/new_tokens.json` file in editor of choice.
- Each key in the file contains a list. Following are the keys:
    - `FUNCTION_CALL`: Function calls and constructor calls used in `ROS1_Package`
    - `TYPECAST_OPERATOR`: Typecast operator functions defined in the `ROS1_Package`
    - `HEADER_FILE`: Included header files in the `ROS1_Package`
    - `MACRO_DEFINITION`: Macros used in `ROS1_Package`
    - `NAMESPACE`: Namespaces used in `ROS1_Package`
    - `FUNCTION_PARAMETER`: Parameters of functions declared in `ROS1_Package`
    - `VARIABLE_TYPE`: Variable types used in `ROS1_Package`

- If the list is not empty, then update the `ros2_name` field for each of of the objects in the list. These can be left
unchanged if no change is required for them in ROS2

- All the elements from the list whose value for `ros2_name` was modified, will be moved to the file with name provided
using `--mapping_file` inside `ros2-migration-tools/mapping` folder. Default file name is `master_mappings.json`.

- All elements from the list with value for `ros2_name` unchanged, i.e. `ros2_name??`, will be moved to `IRRELEVANT_TOKENS`
in the file with name provided using `--filer_out_file` inside `ros2-migration-tools/token_filers` folder.
Default file name is `filtered_out_tokens.json`. These elements will not appear in `NEW_TOKENS_LIST` again.

- Some key categories will also require some additional information which is explained below:
    - `FUNCTION_CALL`:
        - `node_arg_req`: This field will be inside `node_arg_info`. Change it to `true` if the function needs
        `node`(actual var name) as an argument
        - `node_arg_ind`: Index(0-based) of the `node` argument if `node_arg_req` was changed to `true`
    - `MACRO_DEFINITION`:
        - `node_arg_req`: This field will be inside `node_arg_info`. Change it to `true` if the function needs
        `node` member function as an argument
        - `node_arg_ind`: Index(0-based) of the `node` argument if `node_arg_req` was changed to `true`
        - `member_name_if_true`: Name of the member function of ROS2 Node class if `node_arg_req` was changed to `true`
    - `VARIABLE_TYPE`:
        - `to_shared_ptr`: It should be true if the `var_type` is supposed to be `shared_ptr`, `false` otherwise
        - `to_be_removed`: It should be true if any var type needs to be removed
    - `FUNCTION_PARAMETER`: It may contain `&` or `*`, e.g, `ros::NodeHandle &`. Provide `ros2_name` with exactly what you want
        to replace it with

*Note: If there is no change in name from ROS1 to ROS2 but above fields like `node_arg_req` is changed, then change
the field `ros2_name` same as `ros1_name`, i.e. do change it from `ros2_name??` otherwise it will get added to the
`IRRELEVANT_TOKENS`*

*Note: If there is scope resolution included in `ros1_name`(e.g. `ros::ros1_token`), then only change the
`ros1_token` to corresponding `ros2_token`. So `ros2_name` field would look like `ros::ros2_token` and not like `rclcpp::ros2_token`.
 Namespace change will be taken care by `NAMESPACE` category.*
