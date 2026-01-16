////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2026 Matthieu Beauchamp-Boulay
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it freely,
// subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented;
//    you must not claim that you wrote the original software.
//    If you use this software in a product, an acknowledgment
//    in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such,
//    and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
//
////////////////////////////////////////////////////////////


#include "Benchmark.hpp"
#include "Simu/profiler.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

using namespace std::chrono_literals;

struct BenchmarkSettings
{
    const char*          name;
    std::chrono::seconds duration;

    std::unique_ptr<simu::Benchmark> benchmark;
};


int main(int argc, char** argv) {
    namespace fs = std::filesystem;

    const char* output_dir = argc == 2 ? argv[1] : "output";

    if (fs::exists(output_dir)) {
        std::cout << "Overwriting data in " << output_dir << std::endl;
        fs::remove_all(output_dir);
    }
    fs::create_directory(output_dir);

    const BenchmarkSettings benchmarks[] = {
        {"Box stacks (300x10)",   20s, std::make_unique<simu::BoxStacks>(300,  10) },
    };

    for (const auto& benchmark : benchmarks) {
        std::cout << "Running " << benchmark.name << " for "
                  << benchmark.duration << std::endl;

        simu::Simulation simu;
        benchmark.benchmark->init(simu);

        double elapsed = 0.0;
        while (elapsed < benchmark.duration.count()) {
            benchmark.benchmark->step(simu);
            elapsed += simu.settings().dt;
        }

        auto path = fs::path(output_dir) / (std::string(benchmark.name) + ".csv");
        std::ofstream file(path);
        if (!file) {
            std::cerr << "Failed to open " << path << std::endl;
        }

        simu::profiler::write_profiled_data(file);
        std::cout << "    => Results written to " << path << std::endl;

        SIMU_RESET_PROFILER;
    }
}