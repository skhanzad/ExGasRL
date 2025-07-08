#!/bin/bash

# Heuristic Search Framework Build Script
# This script builds the C++ heuristic search framework

set -e  # Exit on any error

echo "Building Heuristic Search Framework..."
echo "====================================="

# Check if CMake is available
if ! command -v cmake &> /dev/null; then
    echo "Error: CMake is not installed. Please install CMake 3.16 or higher."
    exit 1
fi

# Check CMake version
CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
echo "Found CMake version: $CMAKE_VERSION"

# Create build directory
echo "Creating build directory..."
mkdir -p build
cd build

# Configure with CMake
echo "Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build the project
echo "Building project..."
make -j$(nproc)

echo ""
echo "Build completed successfully!"
echo "============================"
echo ""
echo "Available targets:"
echo "  ./bin/search_example  - Run the example program"
echo "  ./bin/search_tests    - Run the test suite"
echo ""
echo "To run tests:"
echo "  make test"
echo ""
echo "To install:"
echo "  make install"
echo ""
echo "To clean build:"
echo "  make clean" 