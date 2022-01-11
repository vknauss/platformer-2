
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "game.h"


enum class init_error {
    NO_ERROR,
    GLFW_INIT_FAILED,
    WINDOW_CREATION_FAILED,
    GLEW_INIT_FAILED
};

static init_error init(GLFWwindow** window) {
    if (!glfwInit()) {
        return init_error::GLFW_INIT_FAILED;
    }

    *window = glfwCreateWindow(1280, 800, "Window", nullptr, nullptr);
    if (!(*window)) {
       return init_error::WINDOW_CREATION_FAILED;
    }

    glfwMakeContextCurrent(*window);

    if (glewInit() != GLEW_OK) {
        return init_error::GLEW_INIT_FAILED;
    }

    return init_error::NO_ERROR;
}

static void print_error(init_error err) {
    switch (err) {
    case init_error::GLFW_INIT_FAILED:
        std::cerr << "Failed to initialize GLFW." << std::endl;
        break;
    case init_error::WINDOW_CREATION_FAILED:
        std::cerr << "Failed to create window." << std::endl;
        break;
    case init_error::GLEW_INIT_FAILED:
        std::cerr << "Failed to initialize GLEW." << std::endl;
        break;
    default:
        break;
    }
}

static void cleanup(init_error err, GLFWwindow* window) {
    switch (err) {
    case init_error::NO_ERROR:
    case init_error::GLEW_INIT_FAILED:
        glfwDestroyWindow(window);
    case init_error::WINDOW_CREATION_FAILED:
        glfwTerminate();
    case init_error::GLFW_INIT_FAILED:
    default:
        break;
    }
}

static void run(GLFWwindow* window) {
    game* g = game::create();

    g->init(window);

    // glfwSwapInterval(1);

    auto time = glfwGetTimerValue();
    auto fps_timer = time;
    int frames = 0;

    game::status status = game::status::running;
    while (status == game::status::running) {
        glfwPollEvents();

        auto lastTime = time;
        time = glfwGetTimerValue();
        float dt = (double) (time - lastTime) / glfwGetTimerFrequency();

        if (time - fps_timer >= glfwGetTimerFrequency()) {
            glfwSetWindowTitle(window, (std::string("FPS: ") + std::to_string(frames)).c_str());
            fps_timer = time;
            frames = 0;
        }
        ++frames;

        status = g->update(dt);

        if (glfwWindowShouldClose(window)) {
            status = game::status::quit;
        }

        int width, height;
        glfwGetWindowSize(window, &width, &height);

        glViewport(0, 0, width, height);

        glClear(GL_COLOR_BUFFER_BIT);

        g->render(width, height);

        glfwSwapBuffers(window);
    }

    delete g;
}

int main() {
    GLFWwindow* window;

    auto err = init(&window);
    
    print_error(err);

    run(window);

    cleanup(err, window);

    return 0;
}