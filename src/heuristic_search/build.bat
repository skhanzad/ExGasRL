@echo off
REM Heuristic Search Framework Build Script for Windows
REM This script builds the C++ heuristic search framework

echo Building Heuristic Search Framework...
echo =====================================

REM Check if CMake is available
cmake --version >nul 2>&1
if errorlevel 1 (
    echo Error: CMake is not installed. Please install CMake 3.16 or higher.
    pause
    exit /b 1
)

REM Check CMake version
for /f "tokens=3" %%i in ('cmake --version') do set CMAKE_VERSION=%%i
echo Found CMake version: %CMAKE_VERSION%

REM Create build directory
echo Creating build directory...
if not exist build mkdir build
cd build

REM Configure with CMake
echo Configuring with CMake...
cmake .. -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 (
    echo Error: CMake configuration failed.
    pause
    exit /b 1
)

REM Build the project
echo Building project...
cmake --build . --config Release
if errorlevel 1 (
    echo Error: Build failed.
    pause
    exit /b 1
)

echo.
echo Build completed successfully!
echo ============================
echo.
echo Available targets:
echo   bin\Release\search_example.exe  - Run the example program
echo   bin\Release\search_tests.exe    - Run the test suite
echo.
echo To run tests:
echo   ctest --verbose
echo.
echo To clean build:
echo   cmake --build . --target clean
echo.
pause 