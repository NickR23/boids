cmake_minimum_required(VERSION 3.20)
project(MetalBoids)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin) # Set the output directory to be in build/bin
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address")


# Metal-cpp headers
include_directories(external/metal-cpp)

# Find required frameworks
find_library(METAL_LIBRARY Metal)
find_library(FOUNDATION_LIBRARY Foundation)
find_library(QUARTZCORE_LIBRARY QuartzCore)


# Find raylib
find_package(raylib REQUIRED)

add_executable(boids src/main.cpp)

# Compile Metal shader to library
find_program(XCRUN xcrun)
if(XCRUN)
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/default.metallib
        COMMAND ${XCRUN} -sdk macosx metal -c
            ${CMAKE_CURRENT_SOURCE_DIR}/shaders/boids.metal
            -o ${CMAKE_CURRENT_BINARY_DIR}/boids.air
        COMMAND ${XCRUN} -sdk macosx metallib
            ${CMAKE_CURRENT_BINARY_DIR}/boids.air
            -o ${CMAKE_CURRENT_BINARY_DIR}/default.metallib
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/boids.metal
        COMMENT "Compiling Metal shader"
    )

    # Copy the metallib to the executable directory
    add_custom_command(
        OUTPUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/default.metallib
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_BINARY_DIR}/default.metallib
            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/default.metallib
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/default.metallib
        COMMENT "Copying Metal library to executable directory"
    )

    add_custom_target(metal_shaders ALL DEPENDS
        ${CMAKE_CURRENT_BINARY_DIR}/default.metallib
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/default.metallib
    )
    add_dependencies(boids metal_shaders)
endif()

target_link_libraries(boids
    raylib
    ${METAL_LIBRARY}
    ${FOUNDATION_LIBRARY}
    ${QUARTZCORE_LIBRARY}
)
