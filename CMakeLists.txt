############################################################
# Project configuration
############################################################

cmake_minimum_required(VERSION 3.13.0)
project(Simu VERSION 0.1.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(macros)

cmake_policy(SET CMP0076 NEW) # target_sources converted to absolute path

set(CMAKE_CXX_STANDARD_REQUIRED 20)
set(CMAKE_CXX_EXTENSIONS OFF)

simu_update_submodules(${CMAKE_CURRENT_LIST_DIR})

############################################################
# Build external libs
############################################################

# add_subdirectory(libs)


############################################################
# Build our components
############################################################

set(simuComponents 
    math
    physics
    graphics
)

set(simuLibs ${simuComponents})
list(TRANSFORM simuLibs PREPEND simu-)

foreach(lib IN LISTS simuLibs)
    simu_add_library(${lib} "")

    target_include_directories(${lib} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
        $<INSTALL_INTERFACE:include>  # <prefix>/include
    )
endforeach()

add_subdirectory(include/Simu)
add_subdirectory(src/Simu)


############################################################
# Tests
############################################################

include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        605a34765aa5d5ecbf476b4598a862ada971b0cc # v3.0.1
)
FetchContent_MakeAvailable(Catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)

# hack to prevent CTest targets from showing up
# https://stackoverflow.com/a/57240389
set_property(GLOBAL PROPERTY CTEST_TARGETS_ADDED 1) 
include(CTest)

include(Catch)

enable_testing()

add_executable(simu-test "")
target_link_libraries(simu-test PRIVATE 
    Catch2::Catch2WithMain
    simu-math
    simu-physics
)
catch_discover_tests(simu-test)

add_subdirectory(test)


############################################################
# Docs
############################################################

add_subdirectory(docs)


############################################################
# TODO: Installation 
############################################################

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)
