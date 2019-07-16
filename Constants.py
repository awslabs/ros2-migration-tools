class AstConstants:
    KIND = "kind"
    NAME = "name"
    NO_SPELLING = "[no spelling]"
    USING_DIRECTIVE = "USING_DIRECTIVE"
    FUNCTION_DECL = "FUNCTION_DECL"
    CALL_EXPR = "CALL_EXPR"
    NAMESPACE_REF = "NAMESPACE_REF"
    VAR_DECL = "VAR_DECL"
    VAR_TYPE = "var_type"
    LINE = "line"
    LINE_TOKENS = "line_tokens"
    DECL_FILEPATH = "declaration_filepath"
    MACRO_INSTANTIATION = "MACRO_INSTANTIATION"
    INCLUSION_DIRECTIVE = "INCLUSION_DIRECTIVE"
    NODE_HANDLE_TYPE = ""
    FIELD_DECL = "FIELD_DECL"

class Constants:
    ROS_1_NAME = "ros1_name"
    ROS_2_NAME = "ros2_name"
    NEW_TOKENS_LIST = "NEW_TOKENS_LIST"
    NEW_MAPPING_MSG = "ros2_name??"
    CONFIG_FILE_NAME = "config.json"
    LIBCLANG_PATH = "LIBCLANG_PATH"
    COMPILE_DB_DIR = "COMPILE_DB_DIR"
    ROS1_SRC_PATH = "ROS1_SRC_PATH"
    ROS2_OUTPUT_DIR = "ROS2_OUTPUT_DIR"
    USER_INCLUDES = "USER_INCLUDES"
    COMPILE_COMMANDS_FILE = "compile_commands.json"
    ROS2_SRC_FOLDER = "src"
    CMAKE_FILE_NAME = "CMakeLists.txt"
    PACKAGE_XML_FILE = "package.xml"
    MAPPING_FILE_NAME = 'mappings.json'
    ROS1_INCLUDE_PATH = "ros/kinetic"
    IRRELEVANT_TOKENS = "IRRELEVANT_TOKENS"
    COLCON_BUILD_SCRIPT_PATH = "./run_colcon_build.sh"
    INCLUDES = "INCLUDES"
    TO_SHARED_PTR = "to_shared_ptr"
    PASS_NODE_VAR = "pass_node_var"
    NODE_ARG_INFO = "node_arg_info"
    NODE_ARG_REQ = "node_arg_req"
    NODE_ARG_INDEX = "node_arg_index"
    MACROS = "MACROS"
    CREATION_INFO = "creation_info"
    IS_CREATED_BY_NODE = "is_created_by_node"
    MEMBER_NAME_IF_TRUE = "member_name_if_true"
    TOKEN_TYPES = (
        AstConstants.CALL_EXPR,
        AstConstants.NAMESPACE_REF,
        AstConstants.VAR_DECL,
        AstConstants.MACRO_INSTANTIATION,
        AstConstants.INCLUSION_DIRECTIVE,
        AstConstants.FIELD_DECL
    )
    MANUAL_TOKENS = (
        INCLUDES
    )
    NODE_NAME = "node_name"
    NODE_HANDLE_VAR_NAME = "node_handle_var_name"
    DEBUGGING = "DEBUG"


class RosConstants:
    INIT_PATTERN = "ros::init"
    INIT_CALL_EXPR = "init"
    NODE_HANDLE = "ros::NodeHandle"
    ROS_INIT_LINE_TOKEN_LENGTH = 10