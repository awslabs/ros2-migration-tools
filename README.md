# ROS Migration Tool

This package contains tools to migrate a ROS1 package to ROS2 package. The "CMakeLists.txt" and "package.xml" migration 
tools from [ros2-migration-tools](https://github.com/awslabs/ros2-migration-tools/tree/master/ros2_migration/porting_tools) 
have been taken as it is. CPP source code migration uses [libclang8](http://releases.llvm.org/download.html) and its 
corresponding [python bindings](https://github.com/llvm-mirror/clang/tree/release_80/bindings/python).

## Pre-requisites
1. ROS1(kinetic) system
2. Python-3.5
3. parse-cmake: Install using `pip3 install parse_cmake` 

## Setup
1. On the ROS1(kinetic) system, clone this repository
2. Open config.json and update:
    - `ROS1_SRC_PATH`: path to ROS1 package src folder
    - `ROS2_OUTPUT_DIR`: path where final migrated package will be copied 

## Usage
1. `cd` to the directory where this repo is cloned

2. Give execution permission to run_colcon_build.sh by running `sudo chmod a+x run_colcon_build.sh`

3. Run `./run_colcon_build`: This will create a copy of the ROS1 package inside `ROS2_OUTPUT_DIR` and compile it with 
`-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

4. On successful run `All mappings filled? Press 'Y' to continue or any other key to abort` will be shown on terminal.
That means compilation was successful. **Don't press 'Y' yet.**

5. [TODO: This step should be an interactive part of the tool.] Open `mappings.json` file in editor of choice. Each key in the file, it contains a list named `NEW_TOKENS_LIST`. If 
it is not empty, then update the `ros2_name` field for all of the objects in the list. All elements from `NEW_TOKENS_LIST`
with value for `ros2_name` as `ros2_name??` will be moved to `IRRELEVANT_TOKENS` so that it don't appear in `NEW_TOKENS_LIST` again.
Some key categories will also require some additional information which is explained below:
    - `CALL_EXPR`:
        - `node_arg_req`: This field will be inside `node_arg_info`. Change it to `true` if the function needs
        `node`(actual var name) as an argument
        - `node_arg_ind`: Index(0-based) of the `node` argument if `node_arg_req` was changed to `true`
    - `MACRO_INSTANTIATION`: 
        - `node_arg_req`: This field will be inside `node_arg_info`. Change it to `true` if the function needs
        `node` member function as an argument
        - `node_arg_ind`: Index(0-based) of the `node` argument if `node_arg_req` was changed to `true`
        - `member_name_if_true`: Name of the member function of ROS2 Node class if `node_arg_req` was changed to `true`
    - `VAR_DECL`:
        - `to_shared_ptr`: It should be true if the `var_type` is supposed to be `shared_ptr`, `false` otherwise  
        
    >Note: If there is no change in name from ROS1 to ROS2 but above fields like `node_arg_req` is changed, then change
    the field `ros2_name` same as `ros1_name`, i.e. do change it from `ros2_name??` otherwise it will get added to the 
    `IRRELEVANT_TOKENS` 
    
    >Note: If there is scope resolution included in `ros1_name`(e.g. `ros::ros1_token`), then only change the 
    `ros1_token` to corresponding `ros2_token`. So `ros2_name` field would look like `ros::ros2_token`. Namespace change
     will be taken care by `NAMESPACE_REF` category.
        
4. `ROS2_OUTPUT_DIR` will have a folder with unique name `currData_currTime`(e.g. 2019-07-15_20_30_55).
    This folder will contain the `src` folder migrated to ROS2 

