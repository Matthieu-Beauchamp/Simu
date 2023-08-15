add_executable(Demos 
    Demos.cpp Demos.hpp
    BoxStacks.cpp 
    Tower.cpp
    Tumbler.cpp 
    Pyramid.cpp 
    NewtonPendulum.cpp
)

target_link_libraries(Demos simu-app)


add_executable(ImGuiTest ImGuiTest.cpp)
target_link_libraries(ImGuiTest simu-app)
