class AstConstants:
    KIND = "kind"
    NAME = "name"
    NO_SPELLING = "[no spelling]"
    USING_DIRECTIVE = "USING_DIRECTIVE"
    FUNCTION_DECL = "FUNCTION_DECL"
    CALL_EXPR = "FUNCTION_CALL"
    NAMESPACE_REF = "NAMESPACE"
    VAR_DECL = "VARIABLE_TYPE"
    VAR_TYPE = "var_type"
    LINE = "line"
    LINE_TOKENS = "line_tokens"
    DECL_FILEPATH = "declaration_filepath"
    MACRO_INSTANTIATION = "MACRO_DEFINITION"
    INCLUSION_DIRECTIVE = "HEADER_FILE"
    NODE_HANDLE_TYPE = ""
    FIELD_DECL = "FIELD_DECL"
    SRC_FILE_PATH = "src_file_path"
    TOKEN_START_COL = "token_start_col"
    TOKEN_END_COL = "token_end_col"
    TYPE_REF = "TYPE_REF"
    MEMBER_REF_EXPR = "MEMBER_REF_EXPR"
    PARM_DECL = "FUNCTION_PARAMETER"
    CONVERSION_FUNCTION = "TYPECAST_OPERATOR"
    DECL_REF_EXPR = "DECL_REF_EXPR"
    CXX_METHOD = "CXX_METHOD"
    CLASS_DECL = "CLASS_DECL"
    UNIT_TEST = "unit_test"
    NON_UNIT_TEST = "non_unit_test"
    TYPE_QUALIFIER = "TYPE_QUALIFIER"
    ROS_VERSIONS = "ROS_VERSIONS"


class Constants:
    ROS_1_NAME = "ros1_name"
    ROS_2_NAME = "ros2_name"
    CONFIG_FILE_NAME = "config.json"
    LIBCLANG_PATH = "clang"
    COMPILE_DB_DIR = "COMPILE_DB_DIR"
    ROS1_SRC_PATH = "ROS1_SRC_PATH"
    ROS2_OUTPUT_DIR = "ROS2_OUTPUT_DIR"
    USER_INCLUDES = "USER_INCLUDES"
    COMPILE_COMMANDS_FILE = "compile_commands.json"
    ROS2_SRC_FOLDER = "src"
    CMAKE_FILE_NAME = "CMakeLists.txt"
    PACKAGE_XML_FILE = "package.xml"
    ROS1_INCLUDE_PATH = "ros/kinetic"
    COLCON_BUILD_SCRIPT_PATH = "./run_colcon_build.sh"
    INCLUDES = "INCLUDES"
    TO_SHARED_PTR = "to_shared_ptr"
    TO_BE_REMOVED = "to_be_removed"
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
        AstConstants.PARM_DECL,
        AstConstants.CONVERSION_FUNCTION
    )
    HELPER_TOKEN_TYPES = {
        AstConstants.MEMBER_REF_EXPR,
        AstConstants.TYPE_REF,
        AstConstants.DECL_REF_EXPR,
        AstConstants.FUNCTION_DECL,
        AstConstants.CXX_METHOD
    }
    MANUAL_TOKENS = (
        INCLUDES
    )
    NODE_NAME = "node_name"
    NODE_HANDLE_VAR_NAME = "node_handle_var_name"
    DEBUGGING = "DEBUG"
    TOKEN_LOCATION = "token_location"
    NODE_VAR_PARENT_CLASS = "node_var_parent_class"
    VOID_CAST = "operator void *"
    REMOVE_VOID_POINTER_CAST = ["ros::ServiceServer"]
    EXTRA_PARAMS = {
        AstConstants.TYPE_REF: {
            "Aws::Utils::Logging::AWSROSLogger": [
                ("Aws::Utils::Logging::LogLevel::Debug", 1)
            ]
        }
    }
    DIFF_FILE_PATH = "migration_diff.txt"
    FILTER_OUT_FILE_PATH = "filter_out.json"
    UNKNOWN_TEMPLATE = "unknown_template"
    DEFAULT_OUTPUT_FOLDER = "output"
    ROS1_PACKAGE_PATH = "ROS1_PACKAGE_PATH"


class RosConstants:
    INIT_PATTERN = "ros::init"
    INIT_CALL_EXPR = "init"
    NODE_HANDLE = "ros::NodeHandle"
    ROS_INIT_LINE_TOKEN_LENGTH = 10
    SERVICE_SERVER = "ros::ServiceServer"
    CREATE_SERVICE = "create_service<>"
    CREATE_SUBSCRIPTION = "create_subscription"
    SUBSCRIBE = "ros::Subscriber"
    TEMPLATED_VARS = {
        SERVICE_SERVER: "advertise_service",
        SUBSCRIBE: "subscribe"
    }
    ROS1_NAMESPACE = "ros::"
    DURATION = "ros::Duration"


class MappingConstants:
    MAPPING_FOLDER = "mapping"
    NEW_MAPPING_MSG = "ros2_name??"
    MASTER_MAPPING_FILE_NAME = 'master_mappings.json'
    MAPPING_TEMPLATE_FILE_NAME = 'mappings_template.json'
    NEW_TOKENS_FILE_NAME = "new_tokens.json"
    NEW_TOKENS_LIST = "NEW_TOKENS_LIST"
    FILTERED_OUT_FILE_NAME = "filtered_out_tokens.json"

    IGNORE_LIST = [
        MAPPING_TEMPLATE_FILE_NAME,
        NEW_TOKENS_FILE_NAME,
        FILTERED_OUT_FILE_NAME
    ]
    IRRELEVANT_TOKENS = "IRRELEVANT_TOKENS"
    TOKEN_FILTERS_FOLDER = "token_filters"


class ClangTokenKind:
    CALL_EXPR = "CALL_EXPR"
    CONVERSION_FUNCTION = "CONVERSION_FUNCTION"
    INCLUSION_DIRECTIVE = "INCLUSION_DIRECTIVE"
    MACRO_INSTANTIATION = "MACRO_INSTANTIATION"
    NAMESPACE_REF = "NAMESPACE_REF"
    PARM_DECL = "PARM_DECL"
    VAR_DECL = "VAR_DECL"
    MEMBER_REF_EXPR = "MEMBER_REF_EXPR"
    TYPE_REF = "TYPE_REF"
    DECL_REF_EXPR = "DECL_REF_EXPR"
    FUNCTION_DECL = "FUNCTION_DECL"
    CXX_METHOD = "CXX_METHOD"
    FIELD_DECL = "FIELD_DECL"
    CLASS_DECL = "CLASS_DECL"