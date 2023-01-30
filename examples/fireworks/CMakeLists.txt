# Collect all header files so that we can add them to the IDE later
file(GLOB_RECURSE SOURCE_LIST CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/examples/fireworks/src/*[.h|.c|.cpp]")
file(GLOB_RECURSE UNITY_SOURCE_LIST CONFIGURE_DEPENDS "${PROJECT_BINARY_DIR}/examples/fireworks/*.cxx")

# Make an automatic library - will be static or dynamic based on user setting
add_executable(fireworks ${SOURCE_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(fireworks PRIVATE ${PROJECT_SOURCE_DIR}/examples/fireworks/src)

# Setup library dependencies
target_link_libraries(fireworks PRIVATE physics)

# IDEs should put the headers in a nice place
set_target_properties(fireworks PROPERTIES FOLDER Examples)
source_group(
        TREE "${PROJECT_SOURCE_DIR}/examples/fireworks/src"
        PREFIX "App"
        FILES ${SOURCE_LIST})

if (PHYSICS_UNITY)
    source_group(
            TREE "${PROJECT_BINARY_DIR}/examples/fireworks/CMakeFiles/fireworks.dir/Unity"
            PREFIX "Unity"
            FILES ${UNITY_SOURCE_LIST})
endif ()
