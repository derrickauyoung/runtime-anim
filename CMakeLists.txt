cmake_minimum_required(VERSION 3.16)
project(RuntimeAnimModule4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- blend library ---
add_library(blend
    src/blend/blend.cpp
)
target_include_directories(blend PUBLIC src)

# --- Executable for demo ---
add_executable(blend_demo src/blend/main.cpp)
target_link_libraries(blend_demo PRIVATE blend)

# --- motion_matching library ---
add_library(motion_matching src/motion_matching/motion_matching.cpp)
target_include_directories(motion_matching PUBLIC src)

add_executable(mm_demo src/motion_matching/main.cpp)
target_link_libraries(mm_demo PRIVATE motion_matching)