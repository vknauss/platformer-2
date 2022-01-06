target_sources(platformer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/game.cpp)

target_include_directories(platformer PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory("shaders")
add_subdirectory("physics")
add_subdirectory("render")