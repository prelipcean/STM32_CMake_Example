# Include Unity configuration
include(${PROJECT_SOURCE_DIR}/cmake/unit_tests.cmake)

# Example test (you can add more tests here)
add_unit_test(test_example ${CMAKE_CURRENT_SOURCE_DIR}/test_example.c)

# Platform types and register utility macros
add_unit_test(test_std_types ${CMAKE_CURRENT_SOURCE_DIR}/test_std_types.c)
