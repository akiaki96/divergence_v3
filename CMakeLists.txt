cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME divergence_v3)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

add_subdirectory(external)

# Maze-solving algorithm library, developed/verified against the desktop
# simulator in external/micromouse_simulator (see that submodule's own
# solver/CMakeLists.txt for details). Kept as a submodule + add_subdirectory
# rather than copied in, so firmware always builds the exact same solver
# sources the simulator verified.
add_subdirectory(external/micromouse_simulator/solver)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u
    _printf_float
)

# CONFIGURE_DEPENDS: re-glob on every build, so a .cpp added by a branch
# switch or merge is compiled without re-running the configure step.
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    Core/Src/app/*.cpp
    Core/Src/device/*.cpp
    Core/Src/algorithm/*.cpp
    Core/Src/menu/*.cpp
    Core/Src/test/*.cpp
    Core/Src/common/*.cpp
)

# Slalom parameters: generated from the designer's JSON (design values) plus
# the hand-tuned deltas, so every build (build.sh, build_flash.sh, IDE) runs
# exactly what the JSON says. Regenerated whenever either JSON or the
# generator changes; the generator needs only the Python standard library.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(GENERATED_INC_DIR ${CMAKE_BINARY_DIR}/generated)
set(SLALOM_PARAMS_HPP ${GENERATED_INC_DIR}/config/slalom_params.hpp)
add_custom_command(
    OUTPUT ${SLALOM_PARAMS_HPP}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/gen_slalom_params.py
            --params ${CMAKE_SOURCE_DIR}/tools/slalom_params.json
            --tuning ${CMAKE_SOURCE_DIR}/tools/slalom_tuning.json
            --out ${SLALOM_PARAMS_HPP}
    DEPENDS
        ${CMAKE_SOURCE_DIR}/tools/gen_slalom_params.py
        ${CMAKE_SOURCE_DIR}/tools/slalom_presets.py
        ${CMAKE_SOURCE_DIR}/tools/slalom_params.json
        ${CMAKE_SOURCE_DIR}/tools/slalom_tuning.json
    COMMENT "Generating config/slalom_params.hpp"
    VERBATIM
)

# Search presets: search speed and which slalom to use, chosen from the
# menu (Run > Search). The generator checks that every preset's slalom is
# designed in slalom_params.json, so a preset naming a missing turn/speed
# stops the build instead of failing on the robot.
set(SEARCH_PRESETS_HPP ${GENERATED_INC_DIR}/config/search_presets.hpp)
add_custom_command(
    OUTPUT ${SEARCH_PRESETS_HPP}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/gen_search_presets.py
            --presets ${CMAKE_SOURCE_DIR}/tools/search_presets.json
            --slalom-params ${CMAKE_SOURCE_DIR}/tools/slalom_params.json
            --out ${SEARCH_PRESETS_HPP}
    DEPENDS
        ${CMAKE_SOURCE_DIR}/tools/gen_search_presets.py
        ${CMAKE_SOURCE_DIR}/tools/gen_slalom_params.py
        ${CMAKE_SOURCE_DIR}/tools/slalom_presets.py
        ${CMAKE_SOURCE_DIR}/tools/slalom_params.json
        ${CMAKE_SOURCE_DIR}/tools/search_presets.json
    COMMENT "Generating config/search_presets.hpp"
    VERBATIM
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${APP_SOURCES}
    ${SLALOM_PARAMS_HPP}
    ${SEARCH_PRESETS_HPP}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    ${GENERATED_INC_DIR}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    etl
    micromouse_solver

    # Add user defined libraries
)
