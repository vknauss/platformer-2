#pragma once

#include <cstdint>


class GLFWwindow;

class game {
public:

    enum class status {
        running, quit
    };

    virtual void init(GLFWwindow* window) = 0;

    virtual status update(float dt) = 0;

    virtual void render(uint32_t width, uint32_t height) = 0;

    static game* create();

    virtual ~game() { }
};