add_executable(Demos2
        Demos.cpp
        BoxStacks.cpp
)
target_link_libraries(Demos2 simu-app)

add_executable(Benchmark
        BenchmarkBoxStacks.cpp
)
target_link_libraries(Benchmark simu-app)

