include(${CMAKE_CURRENT_LIST_DIR}/cmake-modules/CodeCoverage.cmake)

# Taken from https://github.com/SFML/SFML
macro(simu_define_option var default type docstring)
    if (NOT DEFINED ${var})
        set(${var} ${default})
    endif ()

    set(${var} ${${var}} CACHE ${type} ${docstring} FORCE)
endmacro()

function(simu_has_internet boolOut)
    # Avoids CMake configuration failures due to fetchContent when 
    #   not connected to internet
    
    # https://stackoverflow.com/a/62233098
    if (MSVC)
        execute_process(
            COMMAND ping www.google.com /n 2 /w 1000
            ERROR_QUIET
            OUTPUT_QUIET
            RESULT_VARIABLE NO_CONNECTION
        )
    else()
        execute_process(
            COMMAND ping www.google.com -c 2 -w 1000
            ERROR_QUIET
            OUTPUT_QUIET
            RESULT_VARIABLE NO_CONNECTION
        )
    endif()

    if (NO_CONNECTION GREATER 0)
        set(${boolOut} FALSE PARENT_SCOPE)
    else()
        set(${boolOut} TRUE PARENT_SCOPE)
    endif()

    return()
endfunction()

function(simu_update_submodules dir)
    find_package(Git)

    if (${Git_FOUND})
        # TODO: github workflows does not allow us to ping, causing this to not run
        # simu_has_internet(hasInternet)
        # if (${hasInternet})
            execute_process(
                COMMAND "${GIT_EXECUTABLE}" submodule update --init --recursive .
                WORKING_DIRECTORY "${dir}"
            )
        # else()
        #     message(WARNING "Not connected to internet, cannot update submodules")
        # endif()
    else()
        message(WARNING "Could not find git, cannot update submodules")
    endif()
endfunction()

macro(simu_set_compile_options targetName)
	if (MSVC)
    	target_compile_options(${targetName} PRIVATE /nologo /Zi /Zc:__cplusplus /W4 /permissive- )
		string(REGEX REPLACE "(^| )/W[0-4]" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

        if (CMAKE_VERSION VERSION_LESS "3.13.0")
		else()
			target_link_options(${targetName} PRIVATE /nologo)
		endif()
    else()
		target_compile_options(${targetName} PRIVATE 
            -fsigned-char -fanalyzer
            -Wall -Wextra -Wpedantic -Werror
            -Woverloaded-virtual -Wno-unknown-pragmas -Wno-enum-compare
        )
    endif()
endmacro()

macro(simu_coverage targetName)
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
    
endmacro()