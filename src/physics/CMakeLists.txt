target_sources(platformer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/collision.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics.cpp)

target_include_directories(platformer PUBLIC
    ${CMAKE_CURRENT_LIST_DIR})
