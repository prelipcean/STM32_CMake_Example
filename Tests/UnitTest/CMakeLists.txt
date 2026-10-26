# Include Unity configuration
include(${PROJECT_SOURCE_DIR}/cmake/unit_tests.cmake)

# Example test (you can add more tests here)
add_unit_test(test_example ${CMAKE_CURRENT_SOURCE_DIR}/test_example.c)

# Platform types and register utility macros
add_unit_test(test_std_types ${CMAKE_CURRENT_SOURCE_DIR}/test_std_types.c)

# SWC Controller: command replies, with the USART1 BSP faked in the test
add_unit_test(
  test_swc_controller ${CMAKE_CURRENT_SOURCE_DIR}/test_swc_controller.c
  ${PROJECT_SOURCE_DIR}/Project/01_Application/SWC_Controller/Source/swc_controller.c)
target_include_directories(
  test_swc_controller
  PRIVATE ${PROJECT_SOURCE_DIR}/Project/01_Application/SWC_Controller/Include
          ${PROJECT_SOURCE_DIR}/Project/02_VFB/Generated
          ${PROJECT_SOURCE_DIR}/Project/03_Middleware/BSP/USART/Include)

# Runtime measurement: CPU load calculation (header only part)
add_unit_test(test_runtime ${CMAKE_CURRENT_SOURCE_DIR}/test_runtime.c)
target_include_directories(test_runtime PRIVATE ${PROJECT_SOURCE_DIR}/Project/03_Middleware/Runtime/Include)
