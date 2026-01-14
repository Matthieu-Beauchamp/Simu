# Get date
string(TIMESTAMP DATE "%Y-%m-%d")

# Get git commit
execute_process(
        COMMAND git rev-parse HEAD
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

string(SUBSTRING "${GIT_COMMIT}" 0 8 COMMIT_SHORT)

set(OUT_FILE "${DATE}-${COMMIT_SHORT}")
set(OUT_PATH "${RESULTS_DIR}/${OUT_FILE}")

file(MAKE_DIRECTORY "${RESULTS_DIR}")

execute_process(
        COMMAND "${EXE}" "${OUT_PATH}"
        RESULT_VARIABLE RES
)

if(NOT RES EQUAL 0)
    message(FATAL_ERROR "Benchmark failed")
endif()
