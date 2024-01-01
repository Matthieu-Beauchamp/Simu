# Based on https://devblogs.microsoft.com/cppblog/clear-functional-c-documentation-with-sphinx-breathe-doxygen-cmake/
#  and: https://stackoverflow.com/q/34878276
#
# Takes C/C++ headers documented using an extended Doxygen syntax.
# As well as reStructuredText documentation files to document the project.
# 
# Uses Doxygen -> Breathe -> Sphinx

cmake_minimum_required(VERSION 3.0)
project("Docs")

# Taken from https://github.com/SFML/SFML/blob/master/CMakeLists.txt
macro(docs_option var default type docstring)
    if(NOT DEFINED ${var})
        set(${var} ${default})
    endif()
    set(${var} ${${var}} CACHE ${type} ${docstring} FORCE)
endmacro()

docs_option(DOCS_TARGET_NAME "Unnamed-project" STRING "The name of the documented project to use an identifier (Must be a single word)")
docs_option(DOCS_PROJECT_NAME "Unnamed project" STRING "The name of the project as it should appear in the documentation")

docs_option(DOCS_AUTHOR "Anynomous hero" STRING "The name of the author as it should appear in the documentation")
docs_option(DOCS_PROJECT_RELEASE "0.0.1" STRING "The version of the project as it should appear in the documentation")

docs_option(DOCS_HEADER_INPUT_DIR "../include" STRING "The root directory of the project's headers relative to the docs CMakeList")
docs_option(DOCS_INPUT_DIR "source" STRING "The root directory of the project's reStructuredText documentation (sphinx) relative to the docs CMakeList")
docs_option(DOCS_OUTPUT_DIR "build" STRING "Where the documentation will be produced relative to the docs CMakeList")

docs_option(DOCS_LANGUAGE "en" STRING "The language of the generated documentation")
docs_option(DOCS_FORMAT "html" STRING "Any of Sphinx's builders")
docs_option(BUILD_DOCS TRUE BOOL "True to build documentation")

docs_option(DOCS_STRIP_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/.." STRING "The root of the project in order to only show relative paths in the documentation")
docs_option(DOCS_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/${DOCS_HEADER_INPUT_DIR}" STRING "The include directories in order to give proper include in documentation")

docs_option(DOCS_DOXYGEN_OUTPUT_DIR "doxygenOut" STRING "(internal) Where doxygen will produce the XML files for breathe")


# Based on https://stackoverflow.com/q/75551166
# Creates a venv with our required packages
# TODO: Can't we just activate? I think I read that cmake will still use global python interpreter
macro(setupPython)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    if(NOT ${Python3_FOUND})
        message(FATAL_ERROR "Python3 is needed to build the documentation.")
    endif()

    set(VENV "${CMAKE_CURRENT_SOURCE_DIR}/.venv")
    set(REQUIREMENTS "${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt")

    if (WIN32)
        set(BIN_DIR "${VENV}/Scripts")
        set(PIP "${BIN_DIR}/pip.exe")
        set(PY "${BIN_DIR}/python.exe")
        set(SPHINX "${BIN_DIR}/sphinx-build.exe")
        set(BREATHE "${BIN_DIR}/breathe-apidoc.exe")
    else()
        set(BIN_DIR "${VENV}/bin")
        set(PIP "${BIN_DIR}/pip")
        set(PY "${BIN_DIR}/python")
        set(SPHINX "${BIN_DIR}/sphinx-build")
        set(BREATHE "${BIN_DIR}/breathe-apidoc")
    endif()

    add_custom_command(OUTPUT ${PIP} ${PY}
        COMMAND ${Python3_EXECUTABLE} -m venv ${VENV} 
        COMMENT "Creating VENV at ${VENV} using ${Python3_EXECUTABLE}"
    )

    # TODO: Bad output specification, should check site-packages
    #       but on POSIX we have lib/PythonX.XX/site-packages which is troublesome
    add_custom_command(OUTPUT ${SPHINX} ${BREATHE}
        COMMAND ${PIP} install -r ${REQUIREMENTS}
        DEPENDS ${PIP} 
        COMMENT "Installing python packages in VENV"
    )
endmacro()


macro(setupDoxygen)
    find_package(Doxygen)
    if(NOT ${Doxygen_FOUND})
        message(FATAL_ERROR "Doxygen is needed to build the documentation.")
    endif()
    
    set(DOXYGEN_INDEX_FILE ${DOCS_DOXYGEN_OUTPUT_DIR}/xml/index.xml)
    set(DOXYFILE ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(Doxyfile.in ${DOXYFILE} @ONLY)

    file(MAKE_DIRECTORY ${DOCS_DOXYGEN_OUTPUT_DIR}) # Doxygen won't create this for us
endmacro()

macro(generateConf)
    set(CONF_IN ${CMAKE_CURRENT_SOURCE_DIR}/source/conf.py.in)
    set(CONF_OUT ${CMAKE_CURRENT_SOURCE_DIR}/source/conf.py)
   
    file(GLOB_RECURSE HEADERS ${DOCS_HEADER_INPUT_DIR}/*.h*)
    configure_file(${CONF_IN} ${CONF_OUT} @ONLY)
endmacro()

if(${BUILD_DOCS})
    setupPython()
    setupDoxygen()
    generateConf()

    set(INPUT ${CMAKE_CURRENT_SOURCE_DIR}/${DOCS_INPUT_DIR})
    set(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${DOCS_OUTPUT_DIR})

    add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
        DEPENDS ${HEADERS} ${DOXYFILE}
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating ${DOCS_PROJECT_NAME} documentation with Doxygen"
        #VERBATIM
    )

    add_custom_target(${DOCS_TARGET_NAME}-docs ALL
        COMMAND ${SPHINX} -b ${DOCS_FORMAT} 
        -Dbreathe_projects.${DOCS_TARGET_NAME}=${CMAKE_CURRENT_SOURCE_DIR}/${DOCS_DOXYGEN_OUTPUT_DIR}/xml
        -Dbreathe_default_project=${DOCS_TARGET_NAME}
        -Dhtml_theme=sphinx_rtd_theme
        ${INPUT} ${OUTPUT}
        DEPENDS ${SPHINX} ${DOXYGEN_INDEX_FILE} ${CONF_OUT}
        COMMENT "Generating documentation with Sphinx executable: ${SPHINX}\nInput: ${INPUT}\nOutput: ${OUTPUT}"
        #VERBATIM
    )

endif()