foreach(component IN LISTS simuComponents)
    file(GLOB_RECURSE ${component}Src 
        LIST_DIRECTORIES FALSE
        CONFIGURE_DEPENDS
        RELATIVE ${CMAKE_CURRENT_LIST_DIR}
        ${component}/*.cpp
    )
    
    list(APPEND simu_coverage_src ${${component}Src})

    target_sources(simu-${component} PUBLIC ${${component}Src})
endforeach()
