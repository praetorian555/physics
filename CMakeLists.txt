cmake_minimum_required(VERSION 3.20)

project(
        PHYSICS
        VERSION 0.11
        DESCRIPTION "3D Physics Engine with C++"
        LANGUAGES CXX)

# Library options
option(PHYSICS_UNITY "Enable unity builds" ON)
option(PHYSICS_SANITIZER "Enable address sanitizer" ON)
option(PHYSICS_RENDER "Enable built-in renderer" ON)

message(STATUS "PHYSICS_UNITY ${PHYSICS_UNITY}")
message(STATUS "PHYSICS_SANITIZER ${PHYSICS_SANITIZER}")
message(STATUS "PHYSICS_RENDER ${PHYSICS_RENDER}")

# Only do these if this is the main project, and not if it is included through add_subdirectory
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    # Let's ensure -std=c++xx instead of -std=g++xx
    set(CMAKE_CXX_EXTENSIONS OFF)

    include(cmake/optimized-debug.cmake)

    # Configuration types
    SET(CMAKE_CONFIGURATION_TYPES "Debug;OptimizedDebug;Release" CACHE STRING "Choose configuration: Debug OptimizedDebug Release" FORCE)

    # Let's nicely support folders in IDEs
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)

    # Include cmake scripts here
    include(cmake/clang-format.cmake)

    # Global compiler configuration
    if (MSVC)
        # Treat warnings as errors and highest warning level
        add_compile_options(/W4 /WX /wd4201)
        # Multiprocessor compilation
        add_compile_options(/MP)
        # Turn on address sanitizer unless its Release build
        if (PHYSICS_SANITIZER)
            add_compile_options($<$<NOT:$<CONFIG:Release>>:/fsanitize=address>)
        endif ()
    endif ()
    # Use C++ 20 and make it required
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

# Enable unity build
if (PHYSICS_UNITY)
    set(CMAKE_UNITY_BUILD TRUE)
endif ()

include(FetchContent)

# Include math only if it is not already present
if (NOT TARGET math)
    FetchContent_Declare(
            math
            GIT_REPOSITORY https://github.com/praetorian555/math
            GIT_TAG main
    )
    set(MATH_UNITY ${PHYSICS_UNITY} CACHE BOOL "" FORCE)
    set(MATH_SANITIZER ${PHYSICS_SANITIZER} CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(math)
    set_target_properties(math PROPERTIES FOLDER Extern)
endif ()

if (NOT TARGET rndr AND PHYSICS_RENDER)
    FetchContent_Declare(
            rndr
            GIT_REPOSITORY https://github.com/praetorian555/rndr
            GIT_TAG master
    )

    set(RNDR_UNITY ${PHYSICS_UNITY} CACHE BOOL "" FORCE)
    set(RNDR_SANITIZER ${PHYSICS_SANITIZER} CACHE BOOL "" FORCE)
    set(RNDR_ASSIMP ON CACHE BOOL "" FORCE)
    set(RNDR_IMGUI ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(rndr)
    set_target_properties(rndr PROPERTIES FOLDER Extern)
endif ()

# The compiled library code is here
add_subdirectory(src)

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    # Add catch2 dependency
    if (NOT TARGET Catch2)
        FetchContent_Declare(
                Catch2
                GIT_REPOSITORY https://github.com/catchorg/Catch2
                GIT_TAG ab6c7375be9a8e71ee84c6f8537113f9f47daf99 # release-3.2.1
        )
        FetchContent_MakeAvailable(Catch2)
        set_target_properties(Catch2 PROPERTIES FOLDER Extern)
        set_target_properties(Catch2WithMain PROPERTIES FOLDER Extern)
    endif ()

    # Tests
    add_subdirectory(tests)
    enable_testing()
    add_test(NAME main_test COMMAND physics_test)

endif ()
