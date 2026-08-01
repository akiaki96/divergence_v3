# external/CMakeLists.txt

add_library(etl INTERFACE)

target_include_directories(etl
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/etl/include
)