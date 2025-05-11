# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# We have to do this FetchContent dance in the root CMakeLists.txt instead of in `main/CMakeLists.txt` because we want to make sure that
# when ESP-IDF's special script-mode CMake stuff runs we already have the files cloned and ready to go, otherwise we'll fail in the `idf_component_register` call.
include(FetchContent)

FetchContent_Declare(
    ads111x
    SOURCE_DIR     ${CMAKE_SOURCE_DIR}/components/ads111x/
    GIT_REPOSITORY https://github.com/iniw/ads111x.git
    GIT_TAG        1.0.0
    GIT_PROGRESS   TRUE
    GIT_SHALLOW    TRUE
    # See: https://discourse.cmake.org/t/prevent-fetchcontent-makeavailable-to-execute-cmakelists-txt/12704/12
    SOURCE_SUBDIR  path/that/does/not/exist
)

FetchContent_MakeAvailable(ads111x)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(main)
