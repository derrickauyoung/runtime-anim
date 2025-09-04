cmake_minimum_required(VERSION 3.16)
project(MyProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Library / code under test
add_library(mycode src/anim_keys_samples_clips.cpp)
target_include_directories(mycode PUBLIC src)

# Enable CTest so `ctest` works
include(CTest)
enable_testing()

# Add tests subdir
add_subdirectory(tests)
