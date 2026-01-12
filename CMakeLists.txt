############################################################
# Project configuration
############################################################

cmake_minimum_required(VERSION 3.13.0)
project(Simu VERSION 0.1.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(macros)

cmake_policy(SET CMP0076 NEW) # target_sources converted to absolute path

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

simu_update_submodules(${CMAKE_CURRENT_LIST_DIR})

############################################################
# Options
############################################################

simu_define_option(SIMU_BUILD_DEMOS ON BOOL "Build demos")

simu_define_option(SIMU_BUILD_TESTS ON BOOL "Build tests")
simu_define_option(SIMU_TEST_COVERAGE OFF BOOL "Produce coverage report for tests")

simu_define_option(SIMU_BUILD_DOCS OFF BOOL "Build documentation")


############################################################
# Build
############################################################

add_library(simu
        src/Simu/math/BarycentricCoordinates.cpp
        src/Simu/math/Geometry.cpp
        src/Simu/math/Random.cpp

        src/Simu/physics/Body.cpp
        src/Simu/physics/World.cpp

        src/Simu/physics-2.0/collisions.cpp
        src/Simu/physics-2.0/Simulation.cpp
        src/Simu/physics-2.0/BoundingVolumeHierarchy.cpp
        include/Simu/physics-2.0/ObjectBuilder.hpp
        include/Simu/physics-2.0/collision.hpp
        include/Simu/physics-2.0/constraint/contact.hpp
        include/Simu/physics-2.0/constraint/ContactSolverStrategy.hpp
        include/Simu/profiler.hpp
        src/Simu/profiler.cpp
)

target_sources(simu
        PRIVATE FILE_SET HEADERS
        BASE_DIRS include/Simu/
        FILES
        include/Simu/physics-2.0/collision/colliders/ColliderType.hpp
        include/Simu/physics-2.0/collision/CollisionPair.hpp
        include/Simu/physics-2.0/PhysicsObjects/ObjectId.hpp
        include/Simu/physics-2.0/PhysicsObjects/ObjectPool.hpp
        include/Simu/physics-2.0/PhysicsObjects/DynamicPhysicsObject.hpp
        include/Simu/physics-2.0/PhysicsObjects/ColliderOperations.hpp
)

simu_set_compile_options(simu)
simu_set_compile_definitions(simu)
target_compile_definitions(simu PRIVATE SIMU_EXPORT)

target_include_directories(simu PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
        $<INSTALL_INTERFACE:include>  # <prefix>/include
)
target_include_directories(simu PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src>
        $<INSTALL_INTERFACE:src>
)

include(FetchContent)
if (SIMU_BUILD_DEMOS)
    add_library(simu-app
            src/Simu/app/Application.cpp
            src/Simu/app/Camera.cpp
            src/Simu/app/Renderer.cpp
            src/Simu/app/Scene.cpp
            src/Simu/app/Tool.cpp
            include/Simu/app/SimulationRenderer.cpp
    )
    simu_set_compile_options(simu)
    simu_set_compile_definitions(simu)

    target_link_libraries(simu-app simu)

    target_include_directories(simu-app PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
            $<INSTALL_INTERFACE:include>  # <prefix>/include
    )

    set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "" FORCE)
    FetchContent_Declare(
            GLFW
            GIT_REPOSITORY https://github.com/glfw/glfw.git
            GIT_TAG 7482de6 # v3.3.8
    )

    set(OPTION_SELF_CONTAINED OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_TESTS OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_DOCS OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_TOOLS OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_EXAMPLES OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_WITH_BOOST_THREAD OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_CHECK OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_OWN_KHR_HEADERS OFF CACHE INTERNAL "" FORCE)
    set(OPTION_BUILD_WITH_LTO OFF CACHE INTERNAL "" FORCE)
    set(OPTION_USE_GIT_INFORMATION ON CACHE INTERNAL "" FORCE)
    FetchContent_Declare(
            glbinding
            GIT_REPOSITORY https://github.com/cginternals/glbinding.git
            GIT_TAG c8a66c92e8a81d82e384a4c80207913519c1f17e # v3.5.0
    )

    FetchContent_MakeAvailable(GLFW glBinding)
    buildImgui() # Links to glfw, providing symbols

    target_link_libraries(simu-app glbinding::glbinding imgui)

    add_subdirectory(demos)
    add_subdirectory(demos-2)
endif ()

option(SIMU_ENABLE_TRACY "Enable tracy profiler")
option(SIMU_CUSTOM_PROFILER "Enable custom profiler")

if (SIMU_ENABLE_TRACY AND SIMU_CUSTOM_PROFILER)
    message(FATAL_ERROR "Cannot enable tracy and custom profiler together")
endif ()

if (SIMU_ENABLE_TRACY)
    target_compile_definitions(simu PUBLIC TRACY_ENABLE)
    target_compile_definitions(simu PUBLIC SIMU_ENABLE_TRACY)
endif ()

if (SIMU_CUSTOM_PROFILER)
    target_compile_definitions(simu PUBLIC SIMU_CUSTOM_PROFILER)
endif ()

set(TRACY_ENABLE ${SIMU_ENABLE_TRACY})

FetchContent_Declare(
        tracy
        GIT_REPOSITORY https://github.com/wolfpld/tracy.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(tracy)
target_link_libraries(simu PUBLIC TracyClient)

# add_subdirectory(${tracy_SOURCE_DIR}/profiler)

############################################################
# Tests
############################################################

if (SIMU_BUILD_TESTS)

    FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG 3f0283d # v3.3.2
    )
    FetchContent_MakeAvailable(Catch2)
    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)

    # hack to prevent CTest targets from showing up
    # https://stackoverflow.com/a/57240389
    set_property(GLOBAL PROPERTY CTEST_TARGETS_ADDED 1)
    include(CTest)

    include(Catch)

    enable_testing()

    add_executable(simu-test
            test/physics/Collisions.cpp
            test/physics/BoundingVolumeHierarchy.cpp

            test/Interval.cpp
            test/Matrix.cpp
            test/Assertion.cpp
            test/Polygon.cpp
            test/Gjk.cpp
            test/BarycentricCoordinates.cpp

            test/BoundingBox.cpp
            test/RTree.cpp
            test/ContactManifold.cpp
            test/Island.cpp
            test/Physics.cpp
    )

    target_link_libraries(simu-test PRIVATE
            Catch2::Catch2WithMain
            simu
    )

    catch_discover_tests(simu-test)

    if (SIMU_TEST_COVERAGE)
        simu_coverage(simu-test)
    endif ()
endif ()


############################################################
# Docs
############################################################

if (SIMU_BUILD_DOCS)
    set(DOCS_TARGET_NAME " Simu") # Must not have spaces
    set(DOCS_PROJECT_NAME "Simu") # A pretty name to show in documentation

    set(DOCS_AUTHOR "Matthieu Beauchamp") # Your name
    set(DOCS_PROJECT_RELEASE "0.0.1") # Your project's version

    # The following are relative to the docs directory
    set(DOCS_HEADER_INPUT_DIR "../include")
    set(DOCS_INPUT_DIR "source")
    set(DOCS_OUTPUT_DIR "build")

    set(DOCS_LANGUAGE "en") # https://www.sphinx-doc.org/en/master/usage/configuration.html#confval-language
    set(DOCS_FORMAT "html") # https://www.sphinx-doc.org/en/master/usage/builders/index.html

    # When this is true, a ${DOCS_TARGET_NAME}-docs CMake target is available
    set(BUILD_DOCS TRUE)

    # The root directory of your project (absolute directory) to be removed
    # in the documentation
    set(DOCS_STRIP_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}")

    # A comma separated list of include directories (absolute) for the proper #include 
    # directives to appear in the documentation
    set(DOCS_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")

    add_subdirectory(docs)
endif ()

############################################################
# TODO: Installation 
############################################################

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)
