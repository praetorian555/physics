file(GLOB_RECURSE PRIVATE_SOURCE_LIST CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/tests/*[.h|.c|.cpp]")
file(GLOB_RECURSE UNITY_SOURCE_LIST CONFIGURE_DEPENDS "${PROJECT_BINARY_DIR}/tests/*.cxx")

add_executable(physics_test ${PRIVATE_SOURCE_LIST})

target_link_libraries(physics_test PRIVATE Catch2WithMain physics)
set_target_properties(physics_test PROPERTIES FOLDER Physics)

source_group(
  TREE "${PROJECT_SOURCE_DIR}/tests"
  PREFIX "Tests"
  FILES ${PRIVATE_SOURCE_LIST})

if (PHYSICS_UNITY)
  source_group(
    TREE "${PROJECT_BINARY_DIR}/tests/CMakeFiles/physics_test.dir/Unity"
    PREFIX "Unity"
    FILES ${UNITY_SOURCE_LIST})
endif()