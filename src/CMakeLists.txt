# Collect all header files so that we can add them to the IDE later
file(GLOB_RECURSE PUBLIC_SOURCE_LIST CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/include/*.h")
file(GLOB_RECURSE PRIVATE_SOURCE_LIST CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*[.h|.c|.cpp]")
file(GLOB_RECURSE UNITY_SOURCE_LIST CONFIGURE_DEPENDS "${PROJECT_BINARY_DIR}/src/*.cxx")

# Make an automatic library - will be static or dynamic based on user setting
add_library(physics ${PRIVATE_SOURCE_LIST} ${PUBLIC_SOURCE_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(physics PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(physics PRIVATE ${PROJECT_SOURCE_DIR}/src)

# Setup library dependencies
target_link_libraries(physics PUBLIC math)

# IDEs should put the headers in a nice place
set_target_properties(physics PROPERTIES FOLDER Physics)
source_group(
  TREE "${PROJECT_SOURCE_DIR}/include"
  PREFIX "API"
  FILES ${PUBLIC_SOURCE_LIST})
source_group(
  TREE "${PROJECT_SOURCE_DIR}/src"
  PREFIX "Implementation"
  FILES ${PRIVATE_SOURCE_LIST})
if (PHYSICS_UNITY)
  source_group(
    TREE "${PROJECT_BINARY_DIR}/src/CMakeFiles/physics.dir/Unity"
    PREFIX "Unity"
    FILES ${UNITY_SOURCE_LIST})
endif()
