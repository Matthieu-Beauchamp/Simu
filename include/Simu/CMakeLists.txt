foreach(lib IN LISTS simuLibs)
    target_sources(${lib} PUBLIC config.hpp)
endforeach()

list(APPEND simu_coverage_src config.hpp)

foreach(component IN LISTS simuComponents)
    file(GLOB_RECURSE ${component}Headers 
        LIST_DIRECTORIES FALSE
        CONFIGURE_DEPENDS
        RELATIVE ${CMAKE_CURRENT_LIST_DIR}
        ${component}/*.hpp
    )

    list(APPEND simu_coverage_src ${${component}Headers})

    target_sources(simu-${component} PUBLIC ${${component}Headers})
endforeach()
