# B9ROSMigrationTools

This package contains tools to migrate a ROS1 package to ROS2 package. The "CMakeLists.txt" and "package.xml" migration 
tools from [ros2-migration-tools](https://github.com/awslabs/ros2-migration-tools/tree/master/ros2_migration/porting_tools) 
have been taken as it is. CPP source code migration uses [libclang8](http://releases.llvm.org/download.html) and its 
corresponding [python bindings](https://github.com/llvm-mirror/clang/tree/release_80/bindings/python). The main script 
for migration is `B9ROSMigrationTools/ros_upgrader.py`

## Pre-requisites
1. ROS1(kinetic) system
2. Python-3.5 or higher
3. parse-cmake: Install using `pip3 install parse_cmake`

## Setup
1. On the ROS1(kinetic) system, clone `B9ROSMigrationTools` repository:

    `git clone https://code.amazon.com/packages/B9ROSMigrationTools/trees/mainline`
    
2. Download and copy `libclang` files. See [Setup libclang](#setup-libclang) section for more details
   
3. Clone the ROS1 package which you want to port. Lets say the package cloned is `ROS1_Package`. One required argument 
for `B9ROSMigrationTools/ros_upgrader.py` is path to the `package.xml` file of the `ROS1_Package`
  
4. Build the `ROS1_Package`(ROS1 package) with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

    `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`
    
5. Building the `ROS1_Package` with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` must have created a `compile_commands.json` file 
inside `build/ROS1_Package` if the build was successful. This path will be second required argument for `B9ROSMigrationTools/ros_upgrader.py` script

## Setup libclang
1. Download the `LLVM 8.0.0` `Pre-Build Binaries` for your version of linux from [llvm download page](http://releases.llvm.org/download.html).
For e.g., I used Ubuntu 16.06, so I downloaded [Ubuntu 16.04 (.sig)](http://releases.llvm.org/8.0.0/clang+llvm-8.0.0-x86_64-linux-gnu-ubuntu-16.04.tar.xz)
from `Download LLVM 8.0.0` section and `Pre-Built Binaries:` subsection.

2. Extract the binaries

3. Copy the symlink `libclang.so` and lib file `libclang.so.8` from `lib` folder of the extracted folder to `B9ROSMigrationTools\clang` folder

    `clang+llvm-3.9.1-x86_64-linux-gnu-ubuntu-16.04/lib/libclang.so` to `B9ROSMigrationTools\clang`
    
    `clang+llvm-3.9.1-x86_64-linux-gnu-ubuntu-16.04/lib/libclang.so.8` to `B9ROSMigrationTools\clang`


## Usage
1. `cd` to `B9ROSMigrationTools`.

2. run `ros_upgrader.py` script with following arguments.
    - `-c` or `--compile_db_path`(Required): this is path to the `compile_commands.json` file
    - `-p` or `--package_xml_path`(Required): this is path to the `package.xml` file
    - `-o` or `--output_folder`(optional): output directory where the ported package will copied
    - `-d` or `--debug`(optional): add this flag if you want to dump the Abstract Syntax Tree created 

    `ros_upgrader.py -c ROS1_Package/build/ROS1_Package/compile_commands.json -p ROS1_Package/package.xml`
    
    >Note: Paths can also be relative to the `ros_upgrader.py` package

3. One of the following can happen now:
    - If there are no new tokens encountered in the ROS1 package, then it will just finish running 
    - If there are some new tokens encountered, there will be message on terminal saying 
    `Open mapping/new_tokens.json and fill the mappings. Press 'Y' to continue or any other key to abort` 
    will be shown on terminal. **Don't press 'Y' yet.** See the [Filling the mappings](##filling-the-mappings) section 
    on for more details about the mappings
        
3. Default output folder is a folder named `output` inside the `B9ROSMigrationTools` or the output folder will be the folder
    which you specified with `-o` or `--output_folder` argumnet. This folder, either default or custom, will have a folder 
    with unique name `currData_currTime`(e.g. 2019-07-15_20_30_55). This folder will contain the package migrated to ROS2 

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

- If the list is not empty, then update the `ros2_name` field for each of of the objects in the list. 

- All elements from the list with value for `ros2_name` as `ros2_name??` will be moved to `IRRELEVANT_TOKENS` 
in the file `mapping/filtered_out_tokens.json` so that it don't appear in `NEW_TOKENS_LIST` again.

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
        
>Note: If there is no change in name from ROS1 to ROS2 but above fields like `node_arg_req` is changed, then change
the field `ros2_name` same as `ros1_name`, i.e. do change it from `ros2_name??` otherwise it will get added to the 
`IRRELEVANT_TOKENS` 
    
>Note: If there is scope resolution included in `ros1_name`(e.g. `ros::ros1_token`), then only change the 
`ros1_token` to corresponding `ros2_token`. So `ros2_name` field would look like `ros::ros2_token` and not like `rclcpp::ros2_token`.
 Namespace change will be taken care by `NAMESPACE` category.
