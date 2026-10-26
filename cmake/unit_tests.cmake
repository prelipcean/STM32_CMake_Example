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
set(UNITY_CONFIG_DIR ${CMAKE_BINARY_DIR}/unity_config)
file(WRITE ${UNITY_CONFIG_DIR}/unity_config.h [[
#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

void unity_output_char(int c);
void unity_output_flush(void);

#endif /* UNITY_CONFIG_H */
]])

# Add the generated config dir to Unity's include path
target_include_directories(unity PUBLIC $<BUILD_INTERFACE:${UNITY_CONFIG_DIR}>)

# Include directories visible to every test (production headers + mocks)
set(UNIT_TEST_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/Project/YY_Utilities/Include
                           ${PROJECT_SOURCE_DIR}/Tests/UnitTest/mock)

# Function to create test executable
# Usage: add_unit_test(<name> <test.c> [<production sources>...])
function(add_unit_test TEST_NAME TEST_SOURCE)
  add_executable(${TEST_NAME} ${TEST_SOURCE} ${ARGN})
  target_link_libraries(${TEST_NAME} PRIVATE unity)

  # Include application headers and mocks
  target_include_directories(${TEST_NAME} PRIVATE ${UNIT_TEST_INCLUDE_DIRS})

  # Add compiler definitions for testing environment
  target_compile_definitions(${TEST_NAME} PRIVATE UNIT_TESTING RUNNING_ON_HOST)

  # Set compiler flags for testing (GCC/Clang or MSVC host compilers)
  if(MSVC)
    target_compile_options(${TEST_NAME} PRIVATE /W4)
  else()
    target_compile_options(${TEST_NAME} PRIVATE -Wall -Wextra)
  endif()

  # Add test to CTest
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()
