cmake_minimum_required(VERSION 3.10)

project(platformer)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3.3 REQUIRED)

add_executable(platformer "")
add_subdirectory(src)

target_compile_options(platformer PUBLIC "-g" "-fno-omit-frame-pointer" "-fsanitize=address")
target_link_options(platformer PUBLIC "-fno-omit-frame-pointer" "-fsanitize=address")

target_link_libraries(platformer PUBLIC
    OpenGL::GL
    GLEW::GLEW
    glfw)

target_include_directories(platformer PUBLIC
    "../vecmath/include")