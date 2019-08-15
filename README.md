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
2. Build the ROS1 package with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

    `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

## Usage
1. `cd` to the directory where this repo is cloned

2. run `ros_upgrader.py` script with required arguments. One of the following can happen now:
    - If there are no new tokens encountered in the ROS1 package, then it will just finish running 
    - If there are some new tokens encountered, there will be message on terminal saying 
    `Open mappings/new_tokens.json and fill the mappings. Press 'Y' to continue or any other key to abort` 
    will be shown on terminal. **Don't press 'Y' yet.** See the [Filling the mappings](##filling-the-mappings) section 
    on for more details about the mappings
        
3. Default output folder is a folder named `output` inside the `ros_upgrader`. However you can also specify a custom 
    output directory as an argument to the script. This folder, either default or custom, will have a folder with unique name 
    `currData_currTime`(e.g. 2019-07-15_20_30_55). This folder will contain the package migrated to ROS2 

## Filling the mappings
- Open `mappings/new_tokens.json` file in editor of choice. 
- Each key in the file contains a list named. If the list is not empty, then update the `ros2_name` field for all of the 
objects in the list. All elements from the list with value for `ros2_name` as `ros2_name??` will be moved to `IRRELEVANT_TOKENS` 
in the file `mapping/filtered_out_tokens.json` so that it don't appear in `NEW_TOKENS_LIST` again.
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
        - `to_be_removed`: It should be true if any var type needs to be removed
        
>Note: If there is no change in name from ROS1 to ROS2 but above fields like `node_arg_req` is changed, then change
the field `ros2_name` same as `ros1_name`, i.e. do change it from `ros2_name??` otherwise it will get added to the 
`IRRELEVANT_TOKENS` 
    
>Note: If there is scope resolution included in `ros1_name`(e.g. `ros::ros1_token`), then only change the 
`ros1_token` to corresponding `ros2_token`. So `ros2_name` field would look like `ros::ros2_token`. Namespace change
 will be taken care by `NAMESPACE_REF` category.
