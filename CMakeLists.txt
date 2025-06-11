cmake_minimum_required(VERSION 3.20)
project(MetalBoids)

set(CMAKE_CXX_STANDARD 17)

# Metal-cpp headers
include_directories(external/metal-cpp)

# Find required frameworks
find_library(METAL_LIBRARY Metal)
find_library(FOUNDATION_LIBRARY Foundation)
find_library(QUARTZCORE_LIBRARY QuartzCore)

# Find raylib
find_package(raylib REQUIRED)

add_executable(boids src/main.cpp)

target_link_libraries(boids
    raylib
    ${METAL_LIBRARY}
    ${FOUNDATION_LIBRARY}
    ${QUARTZCORE_LIBRARY}
)
