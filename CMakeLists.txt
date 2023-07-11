############################################################
# Project configuration
############################################################

cmake_minimum_required(VERSION 3.13.0)
project(Simu VERSION 0.1.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(macros)

cmake_policy(SET CMP0076 NEW) # target_sources converted to absolute path

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

simu_update_submodules(${CMAKE_CURRENT_LIST_DIR})

############################################################
# Options
############################################################

simu_define_option(SIMU_BUILD_DEMOS   ON BOOL  "Build demos")

simu_define_option(SIMU_BUILD_TESTS   ON BOOL  "Build tests")
simu_define_option(SIMU_TEST_COVERAGE OFF BOOL "Produce coverage report for tests")

simu_define_option(SIMU_BUILD_DOCS    OFF BOOL "Build documentation")


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
)

if (SIMU_BUILD_DEMOS)
    list(APPEND simuComponents app)
endif()

set(simuLibs ${simuComponents})
list(TRANSFORM simuLibs PREPEND simu-)

foreach(lib IN LISTS simuLibs)
    simu_add_library(${lib} "")

    target_include_directories(${lib} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
        $<INSTALL_INTERFACE:include>  # <prefix>/include
    )
endforeach()

include(FetchContent)
if (SIMU_BUILD_DEMOS)

    set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "" FORCE)
    FetchContent_Declare(
        GLFW
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG        7482de6 # v3.3.8
    )

    set(OPTION_SELF_CONTAINED          OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_TESTS             OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_DOCS              OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_TOOLS             OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_EXAMPLES          OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_WITH_BOOST_THREAD OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_CHECK             OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_OWN_KHR_HEADERS   OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_WITH_LTO          OFF CACHE INTERNAL "" FORCE)
    set(OPTION_USE_GIT_INFORMATION     ON  CACHE INTERNAL "" FORCE)
    FetchContent_Declare(
        glbinding
        GIT_REPOSITORY https://github.com/cginternals/glbinding.git
        GIT_TAG        aedb549 # v3.3.0
    )

    FetchContent_MakeAvailable(GLFW glBinding)

    target_link_libraries(simu-app glfw glbinding::glbinding)
    
    add_subdirectory(demos)
endif()


add_subdirectory(include/Simu)
add_subdirectory(src/Simu)

target_link_libraries(simu-physics simu-math)
target_link_libraries(simu-app     simu-physics)


############################################################
# Tests
############################################################

if (SIMU_BUILD_TESTS)

    FetchContent_Declare(
      Catch2
      GIT_REPOSITORY https://github.com/catchorg/Catch2.git
      GIT_TAG        3f0283d # v3.3.2
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
    
    if (SIMU_TEST_COVERAGE)
        simu_coverage(simu-test)
    endif()
endif()


############################################################
# Docs
############################################################

if (SIMU_BUILD_DOCS)
    add_subdirectory(docs)
endif()

############################################################
# TODO: Installation 
############################################################

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)
