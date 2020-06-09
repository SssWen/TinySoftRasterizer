cmake_minimum_required(VERSION 3.5)


# Specify CMake version 3.5 to enable:
#   CMP0028 - Double colon in target name means ALIAS or IMPORTED target.

project(TinySoftRenderring VERSION 1.0.0)
cmake_policy(VERSION 3.5)


# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

add_subdirectory(DrawLineBase)
add_subdirectory(Wireframe)
#add_subdirectory(library)
