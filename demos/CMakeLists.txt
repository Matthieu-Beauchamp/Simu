file(GLOB_RECURSE demos
    LIST_DIRECTORIES FALSE
    CONFIGURE_DEPENDS
    RELATIVE ${CMAKE_CURRENT_LIST_DIR}
    *.cpp
)


foreach(demo IN LISTS demos)
    get_filename_component(demoName ${demo} NAME_WLE)
    add_executable(${demoName} ${demo})

    target_link_libraries(${demoName} simu-app)

endforeach()
