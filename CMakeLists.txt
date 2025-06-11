cmake_minimum_required(VERSION 3.20)
project(boids)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add raylib (assumes installed via Homebrew in default /opt/homebrew/)
find_package(raylib REQUIRED)

# Add executable
add_executable(boids main.cpp)

# Link libraries
target_link_libraries(boids PRIVATE raylib)

# In case you need extra system libs (on macOS, raylib often needs these)
if(APPLE)
  target_link_libraries(boids PRIVATE
    "-framework OpenGL"
    "-framework Cocoa"
    "-framework IOKit"
    "-framework CoreVideo"
  )
endif()
