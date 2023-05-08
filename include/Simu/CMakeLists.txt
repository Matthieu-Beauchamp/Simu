target_sources(simu-math PUBLIC config.hpp)

foreach(component IN LISTS simuComponents)
    file(GLOB_RECURSE ${component}Headers 
        LIST_DIRECTORIES FALSE
        CONFIGURE_DEPENDS
        RELATIVE ${CMAKE_CURRENT_LIST_DIR}
        ${component}/*.hpp
    )

    target_sources(simu-${component} PUBLIC ${${component}Headers})
endforeach()
