# Ensure C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Test executable
add_executable(unit_tests
    test_clip.cpp
)

# Include path for catch_amalgamated.hpp
target_include_directories(unit_tests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src   # if you need your code headers
)

# Link your library/code under test
target_link_libraries(unit_tests PRIVATE mycode)

# Register with CTest
enable_testing()
add_test(NAME unit_tests COMMAND unit_tests)
