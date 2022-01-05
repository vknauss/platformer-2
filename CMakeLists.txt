cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project(platformer)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3.3 REQUIRED)

add_executable(platformer "")
add_subdirectory(src)

if(UNIX AND CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(platformer PUBLIC $<$<CONFIG:DEBUG>:-fno-omit-frame-pointer -fsanitize=address>)
    target_link_options(platformer PUBLIC $<$<CONFIG:DEBUG>:-fno-omit-frame-pointer -fsanitize=address>)
endif()

set(OPENGL_UTILS_PATH "../opengl-utils")
target_include_directories(platformer PUBLIC ${OPENGL_UTILS_PATH}/include)
target_link_directories(platformer PUBLIC ${OPENGL_UTILS_PATH}/build/${CMAKE_BUILD_TYPE})
target_link_libraries(platformer PUBLIC "opengl-utils")

target_link_libraries(platformer PUBLIC
    OpenGL::GL
    GLEW::GLEW
    glfw)

target_include_directories(platformer PUBLIC
    "../vecmath/include")