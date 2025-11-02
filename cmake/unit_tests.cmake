# Unity Test Framework configuration
include(FetchContent)

# Download Unity framework
FetchContent_Declare(
  unity
  GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
  GIT_TAG v2.5.2)

# Make Unity available
FetchContent_MakeAvailable(unity)

# Configure Unity options
target_compile_definitions(
  unity PUBLIC UNITY_OUTPUT_CHAR=unity_output_char
               UNITY_OUTPUT_FLUSH=unity_output_flush UNITY_INCLUDE_CONFIG_H)

# Create Unity config header
file(WRITE ${CMAKE_BINARY_DIR}/unity_config.h [[
#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

void unity_output_char(int c);
void unity_output_flush(void);

#endif /* UNITY_CONFIG_H */
]])

# Add binary dir to Unity's include path
target_include_directories(unity PUBLIC ${CMAKE_BINARY_DIR})

# Function to create test executable
function(add_unit_test TEST_NAME TEST_SOURCE)
  add_executable(${TEST_NAME} ${TEST_SOURCE})
  target_link_libraries(${TEST_NAME} PRIVATE unity)

  # Include application headers and mocks
  target_include_directories(
    ${TEST_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/Application/Include
                         ${CMAKE_SOURCE_DIR}/tests/mock)

  # Add compiler definitions for testing environment
  target_compile_definitions(${TEST_NAME} PRIVATE UNIT_TESTING RUNNING_ON_HOST)

  # Set compiler flags for testing
  target_compile_options(
    ${TEST_NAME} PRIVATE -Wall -Wextra -g $<$<CONFIG:Debug>:-O0>
                         $<$<CONFIG:Release>:-O2>)

  # Add test to CTest
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()
