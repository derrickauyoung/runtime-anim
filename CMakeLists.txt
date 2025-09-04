cmake_minimum_required(VERSION 3.16)
project(RuntimeAnimModule4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Module4 library ---
add_library(blend
    src/blend/blend.cpp
)
target_include_directories(blend PUBLIC src)

# --- Executable for demo ---
add_executable(blend_demo src/blend/main.cpp)
target_link_libraries(blend_demo PRIVATE blend)
