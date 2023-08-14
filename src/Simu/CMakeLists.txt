foreach(component IN LISTS simuComponents)
    file(GLOB_RECURSE ${component}Src 
        LIST_DIRECTORIES FALSE
        CONFIGURE_DEPENDS
        RELATIVE ${CMAKE_CURRENT_LIST_DIR}
        ${component}/*.cpp
    )

    target_sources(simu-${component} PRIVATE ${${component}Src})
endforeach()
