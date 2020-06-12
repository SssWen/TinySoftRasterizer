cmake_minimum_required(VERSION 3.5)

project(TinySoftRenderring VERSION 1.0.0)
cmake_policy(VERSION 3.5)


# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

add_subdirectory(1-DrawLineBase)
add_subdirectory(1-Wireframe)
add_subdirectory(2-DrawTriangle)
add_subdirectory(2-DrawTriangleBarycentric)
add_subdirectory(2-DrawFlatModel)
add_subdirectory(3-zBuffer)
add_subdirectory(4-Transformation)
#add_subdirectory(library)
