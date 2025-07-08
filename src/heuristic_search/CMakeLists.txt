cmake_minimum_required(VERSION 3.16)
project(HeuristicSearch)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/node.cpp
    src/search_algorithm.cpp
    src/astar.cpp
    src/dijkstra.cpp
    src/best_first.cpp
    src/ida_star.cpp
    src/grid_world.cpp
    src/puzzle_state.cpp
)

# Header files
set(HEADERS
    include/node.h
    include/search_algorithm.h
    include/astar.h
    include/dijkstra.h
    include/best_first.h
    include/ida_star.h
    include/grid_world.h
    include/puzzle_state.h
)

# Create library
add_library(heuristic_search STATIC ${SOURCES} ${HEADERS})

# Create executable for examples
add_executable(search_example examples/search_example.cpp)
target_link_libraries(search_example heuristic_search)

# Create executable for tests
add_executable(search_tests tests/test_search.cpp)
target_link_libraries(search_tests heuristic_search)

# Enable testing
enable_testing()
add_test(NAME SearchTests COMMAND search_tests)

# Installation
install(TARGETS heuristic_search
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${HEADERS} DESTINATION include/heuristic_search) 