cmake_minimum_required(VERSION 3.20)

project(
        PHYSICS
        VERSION 0.1.0
        DESCRIPTION "3D Physics Engine with C++"
        LANGUAGES CXX)

# Library options
option(PHYSICS_HARDENING "Enable address sanitizer" ON)
option(PHYSICS_REAL_AS_DOUBLE "Use double-precision floating point numbers" OFF)
option(PHYSICS_BUILD_SAMPLES "Build sample projects" ON)

message(STATUS "PHYSICS_HARDENING ${PHYSICS_HARDENING}")
message(STATUS "PHYSICS_REAL_AS_DOUBLE ${PHYSICS_REAL_AS_DOUBLE}")
message(STATUS "PHYSICS_BUILD_SAMPLES ${PHYSICS_BUILD_SAMPLES}")

# Don't use C++ 20 modules
set(CMAKE_CXX_SCAN_FOR_MODULES 0)

## C++ language configuration boilerplate, hide symbols by default
if (NOT DEFINED CMAKE_CXX_VISIBILITY_PRESET AND NOT DEFINED CMAKE_VISIBILITY_INLINES_HIDDEN)
    set(CMAKE_CXX_VISIBILITY_PRESET hidden)
    set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)
endif ()

# Configure desired compilation options and warnings
include(cmake/compiler-warnings.cmake)
include(cmake/compiler-options.cmake)
add_library(physics_warnings INTERFACE)
add_library(physics_options INTERFACE)
setup_compiler_warnings(physics_warnings)
setup_compiler_options(physics_options)

if (PHYSICS_HARDENING)
    include(cmake/sanitizers.cmake)
    setup_sanitizers(physics_options)
endif ()

include(FetchContent)
if (NOT TARGET opal)
    FetchContent_Declare(
            opal
            GIT_REPOSITORY https://github.com/praetorian555/opal
            GIT_TAG main
    )
    set(OPAL_HARDENING ${PHYSICS_HARDENING} CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(opal)
endif ()

set(PHYSICS_FILES
        ${PROJECT_SOURCE_DIR}/include/physics/core.hpp
        ${PROJECT_SOURCE_DIR}/include/physics/shape.hpp
        ${PROJECT_SOURCE_DIR}/include/physics/body.hpp
        ${PROJECT_SOURCE_DIR}/include/physics/scene.hpp
        ${PROJECT_SOURCE_DIR}/include/physics/intersect.hpp
        ${PROJECT_SOURCE_DIR}/src/core.cpp
        ${PROJECT_SOURCE_DIR}/src/scene.cpp
        ${PROJECT_SOURCE_DIR}/src/intersect.cpp
        ${PROJECT_SOURCE_DIR}/src/shape.cpp
        ${PROJECT_SOURCE_DIR}/src/body.cpp)
add_library(physics ${PHYSICS_FILES})
target_link_libraries(physics PRIVATE physics_options physics_warnings opal)
target_compile_features(physics PRIVATE cxx_std_20)
target_include_directories(physics PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(physics PRIVATE ${PROJECT_SOURCE_DIR}/samples)

if (PHYSICS_BUILD_SAMPLES)

    if (NOT TARGET rndr)
        FetchContent_Declare(
                rndr
                GIT_REPOSITORY https://github.com/praetorian555/rndr
                GIT_TAG master
        )
        set(RNDR_HARDENING ${PHYSICS_HARDENING} CACHE BOOL "" FORCE)
        set(RNDR_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(rndr)
    endif ()

    add_executable(physics_sample_spheres
            samples/shared/player-controller.hpp
            samples/shared/player-controller.cpp
            samples/shared/sample-app.hpp
            samples/shared/sample-app.cpp
            samples/sample-spheres/sample-spheres.cpp)
    target_link_libraries(physics_sample_spheres PRIVATE physics rndr physics_options physics_warnings)
    target_compile_features(physics_sample_spheres PRIVATE cxx_std_20)
    target_include_directories(physics_sample_spheres PRIVATE ${PROJECT_SOURCE_DIR}/samples)

endif()
