cmake_minimum_required(VERSION 3.13)

# Include Unity configuration
include(${CMAKE_SOURCE_DIR}/cmake/unit_tests.cmake)

# Enable testing
enable_testing()

# Create test runner directory
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/runners)

# Example test (you can add more tests here)
add_unit_test(
    test_example
    ${CMAKE_CURRENT_SOURCE_DIR}/test_example.c
)