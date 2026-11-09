cmake_minimum_required(VERSION 3.8)
project(mas248_ur5e_real)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(CANONICAL_XACRO "${CMAKE_CURRENT_SOURCE_DIR}/../mas248_ur5e/urdf/ur5e_workcell.urdf.xacro")
set(DERIVED_XACRO "${CMAKE_CURRENT_BINARY_DIR}/urdf/ur5e_workcell_real.urdf.xacro")
set(DERIVED_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/config/ur5e")
if(NOT EXISTS "${CANONICAL_XACRO}")
  message(FATAL_ERROR "Expected MAS248 Lecture 8 workcell: ${CANONICAL_XACRO}")
endif()
# This generator writes ONLY into build/, never to any Lecture 8/9 source.
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/prepare_description.py"
          --source "${CANONICAL_XACRO}" --output "${DERIVED_XACRO}"
          --config-output "${DERIVED_CONFIG}"
  RESULT_VARIABLE GENERATION_CODE
  OUTPUT_VARIABLE GENERATION_OUT ERROR_VARIABLE GENERATION_ERR)
if(NOT GENERATION_CODE EQUAL 0)
  message(FATAL_ERROR "Cannot generate Lecture 10 workcell: ${GENERATION_ERR}")
endif()
message(STATUS "${GENERATION_OUT}")
# Source configuration is visible inside the ZIP; generated configs are vendor files.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config/" DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/config/" DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/urdf" "${CMAKE_CURRENT_SOURCE_DIR}/launch"
        DESTINATION share/${PROJECT_NAME})
foreach(script validate_description.py validate_stack.py run_picture_real.py run_picture_physical.py preflight.py)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${script}")
    install(PROGRAMS "scripts/${script}" DESTINATION lib/${PROJECT_NAME})
  else()
    message(STATUS "Not supplied yet: ${script}")
  endif()
endforeach()
install(FILES scripts/common.py scripts/picture_planning.py scripts/test_ursim_motion.py
        DESTINATION lib/${PROJECT_NAME})
ament_package()
