list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake-modules")

# Taken from https://github.com/SFML/SFML
macro(simu_define_option var default type docstring)
    if (NOT DEFINED ${var})
        set(${var} ${default})
    endif ()

    set(${var} ${${var}} CACHE ${type} ${docstring} FORCE)
endmacro()

function(simu_update_submodules dir)
    find_package(Git)

    if (${Git_FOUND})
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" submodule update --init --recursive .
            WORKING_DIRECTORY "${dir}"
            RESULT_VARIABLE status
        )
        if(status AND NOT status EQUAL 0)
            message(STATUS "Could not update submodules: ${status}")
        endif()
    else()
        message(WARNING "Could not find git, cannot update submodules")
    endif()
endfunction()

# https://dirtyhandscoding.github.io/posts/fast-debug-in-visual-c.html
if (MSVC)
    string(REGEX REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
endif()

macro(simu_set_compile_options targetName)
	if (MSVC)
    	target_compile_options(${targetName} PRIVATE 
            /diagnostics:caret 
            /nologo 
            /W4 /WX
            /permissive-
            # $<$<CONFIG:Debug>: /Ob1 /Zo>
        )

        if (CMAKE_VERSION VERSION_LESS "3.13.0")
		else()
			target_link_options(${targetName} PRIVATE /nologo)
		endif()
    else()
		target_compile_options(${targetName} PRIVATE 
            -Wall -Wextra -Wpedantic -Werror
            -Woverloaded-virtual -Wno-unknown-pragmas -Wno-enum-compare
            -fsigned-char
        )
    endif()

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(${targetName} PRIVATE 
            $<$<CONFIG:Debug>: -Og>
        )
    endif()
endmacro()

macro(simu_coverage targetName)
    include(CodeCoverage)

    if (NOT MSVC)
        target_compile_options(${targetName} PRIVATE --coverage)
        if (CMAKE_VERSION VERSION_LESS "3.13.0")
            set_target_properties(${targetName} PROPERTIES COMPILE_OPTIONS --coverage)
        else()
            target_link_options(${targetName} PRIVATE --coverage)
        endif()
        
        setup_target_for_coverage_lcov(NAME ${targetName}-coverage 
            EXECUTABLE ${targetName}
            EXCLUDE "test/*" "*/catch2/*"
        )

        add_custom_target(${targetName}-coverage-viewer
            COMMAND firefox ${PROJECT_BINARY_DIR}/${targetName}-coverage/index.html
            DEPENDS ${targetName}-coverage
        )
    endif()
endmacro()

macro(simu_set_compile_definitions targetName)
    if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        target_compile_definitions(${targetName} PUBLIC SIMU_OS_WINDOWS)
    elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        target_compile_definitions(${targetName} PUBLIC SIMU_OS_LINUX)
    elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        target_compile_definitions(${targetName} PUBLIC SIMU_OS_MACOSX)
    endif()
endmacro()

macro(simu_add_library targetName src...)
    set(src ${src...} ${ARGN})

    add_library(${targetName} ${src})
    simu_set_compile_options(${targetName})
    simu_set_compile_definitions(${targetName})
    target_compile_definitions(${targetName} PRIVATE SIMU_EXPORT)
endmacro()

function(buildImgui)
    # https://stackoverflow.com/a/65620000
    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui
        GIT_TAG        docking
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
    )

    FetchContent_GetProperties(imgui)
    if(NOT imgui_POPULATED)
        FetchContent_Populate(imgui)
    endif()

    add_library(imgui "")
    target_link_libraries(imgui glfw)

    target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR})

    set(imguiSrc
        imconfig.h
        imgui.h
        imgui.cpp

        imgui_demo.cpp
        imgui_draw.cpp
        imgui_tables.cpp
        imgui_widgets.cpp
    
        backends/imgui_impl_glfw.h
        backends/imgui_impl_glfw.cpp
        backends/imgui_impl_opengl3.h
        backends/imgui_impl_opengl3.cpp
    )

    list(TRANSFORM imguiSrc PREPEND "${imgui_SOURCE_DIR}/")
    target_sources(imgui PRIVATE ${imguiSrc})

endfunction()