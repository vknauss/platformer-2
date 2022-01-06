
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <ogu/shader.h>
#include <ogu/vertex_array.h>

#include <GLFW/glfw3.h>

#include <vvm/vvm.hpp>
#include <vvm/matrix_tfm.hpp>

#include "physics/physics.h"
#include "util.h"
#include "render/pass.h"


enum ErrorCode {
    NO_ERROR,
    GLFW_INIT_FAILED,
    WINDOW_CREATION_FAILED,
    GLEW_INIT_FAILED
};

struct App {
    GLFWwindow* window;
    bool isRunning;
};

ErrorCode init(App& app) {
    app.isRunning = false;

    if (!glfwInit()) {
        return GLFW_INIT_FAILED;
    }

    app.window = glfwCreateWindow(1280, 800, "Window", nullptr, nullptr);
    if (!app.window) {
       return WINDOW_CREATION_FAILED;
    }

    glfwMakeContextCurrent(app.window);

    if (glewInit() != GLEW_OK) {
        return GLEW_INIT_FAILED;
    }

    app.isRunning = true;

    return NO_ERROR;
}

void printError(ErrorCode err) {
    switch (err) {
    case GLFW_INIT_FAILED:
        std::cerr << "Failed to initialize GLFW." << std::endl;
        break;
    case WINDOW_CREATION_FAILED:
        std::cerr << "Failed to create window." << std::endl;
        break;
    case GLEW_INIT_FAILED:
        std::cerr << "Failed to initialize GLEW." << std::endl;
        break;
    default:
        break;
    }
}

void cleanup(ErrorCode err, App& app) {
    switch (err) {
    case NO_ERROR:
    case GLEW_INIT_FAILED:
        glfwDestroyWindow(app.window);
    case WINDOW_CREATION_FAILED:
        glfwTerminate();
    case GLFW_INIT_FAILED:
    default:
        break;
    }
}

#define SHADER_PATH(X) SHADERS_DIR "/" X

struct keyboard_state {
    std::map<int, bool> key_down;
    std::map<int, bool> key_pressed;
};

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    auto ks = (keyboard_state*) glfwGetWindowUserPointer(window);
    if (action == GLFW_PRESS) {
        ks->key_down[key] = true;
        ks->key_pressed[key] = true;
    } else if (action == GLFW_RELEASE) {
        ks->key_down[key] = false;
    }
}

void run(App& app) {
    keyboard_state ks;
    glfwSetWindowUserPointer(app.window, &ks);
    glfwSetKeyCallback(app.window, key_callback);

    ogu::buffer vbo(10 * sizeof(vvm::v2f));

    ogu::vertex_buffer_binding vbo_binding {vbo};
    vbo_binding.instanced = false;
    vbo_binding.stride = sizeof(vvm::v2f);
    vbo_binding.attribs.push_back({.location = 0, .size = 2, .type = GL_FLOAT});

    ogu::vertex_array vao({vbo_binding});
    
    vbo.write(0, 0, [] (void* ptr) {
        vvm::v2f* vptr = (vvm::v2f*) ptr;
        *(vptr++) = {-0.5, -0.5};
        *(vptr++) = { 0.5, -0.5};
        *(vptr++) = {-0.5,  0.5};
        *(vptr++) = { 0.5,  0.5};
        *(vptr++) = {-0.5, -0.5};
        *(vptr++) = {-0.5,  0.5};
        *(vptr++) = { 0.5, -0.5};
        *(vptr++) = { 0.5,  0.5};
        *(vptr++) = { 0.0,  0.0};
        *(vptr++) = { 0.5,  0.0};
    });

    ogu::shader_program program({
        {{utils::file_string(SHADER_PATH("vert.glsl"))}, ogu::shader::type::VERTEX},
        {{utils::file_string(SHADER_PATH("frag.glsl"))}, ogu::shader::type::FRAGMENT} });
    // program.addUniform("mvp");
    program.addUniformBuffer("scene_data");
    program.addUniform("color");

    // init physics
    physics::dynamics_world pworld;

    // create some collision shapes
    auto box_shape_id = pworld.add_collision_shape({.extents = {0.5, 0.5}});
    auto ground_shape_id = pworld.add_collision_shape({.extents = {10, 1}});
    auto table_shape_id = pworld.add_collision_shape({.extents = {1.5, 0.05}});
    auto table_leg_shape_id = pworld.add_collision_shape({.extents = {0.15, 0.5}});

    // 0 mass rigid bodies are static / kinematic: 1-way interaction with dynamic bodies
    auto player_id = pworld.add_rigid_body(1, {.position = {0, 0}, .angle = 0}, box_shape_id, 0.5);

    // positive mass rigid bodies are dynamic
    pworld.add_rigid_body(1, {.position = {5, 0}}, box_shape_id, 0.5);
    pworld.add_rigid_body(1, {.position = {5, 2.5}}, box_shape_id, 0.5);
    pworld.add_rigid_body(1, {.position = {5, 5.0}}, box_shape_id, 0.5);
    pworld.add_rigid_body(1, {.position = {5, 8.0}}, box_shape_id, 0.5);
    pworld.add_rigid_body(1, {.position = {5, 11.0}}, box_shape_id, 0.5);
    pworld.add_rigid_body(1, {.position = {5, 14.0}}, box_shape_id, 0.5);
    
    // 0 mass bodies are static
    pworld.add_rigid_body(0, {.position = {0, -4}, .angle = 0.0}, ground_shape_id, 0.5);

    // add a table
    pworld.add_rigid_body(0.1, {.position = {-1.4, -2.5}}, table_leg_shape_id, 0.5);
    pworld.add_rigid_body(0.1, {.position = { 1.4, -2.5}}, table_leg_shape_id, 0.5);
    pworld.add_rigid_body(0.1, {.position = { 0.0, -1.95}}, table_shape_id, 0.5);

    // glUseProgram(program);
    program.use();

    glfwSwapInterval(1);

    ogu::buffer scene_data_buffer(sizeof(vvm::m2f));

    auto time = glfwGetTimerValue();
    bool pause = false;
    bool draw_extra = false;
    while (app.isRunning) {
        ks.key_pressed.clear();
        glfwPollEvents();

        if (glfwWindowShouldClose(app.window) || ks.key_down[GLFW_KEY_ESCAPE]) {
            app.isRunning = false;
        }

        int width, height;
        glfwGetWindowSize(app.window, &width, &height);

        auto lastTime = time;
        time = glfwGetTimerValue();
        float dt = (double) (time - lastTime) / glfwGetTimerFrequency(); 

        if (ks.key_pressed[GLFW_KEY_P]) {
            pause = !pause;
        }
        
        if (ks.key_pressed[GLFW_KEY_E]) {
            draw_extra = !draw_extra;
        }

        vvm::v2f move_dir(0, 0);
        if (ks.key_down[GLFW_KEY_W]) {
            move_dir.y += 1;
        }
        if (ks.key_down[GLFW_KEY_S]) {
            move_dir.y -= 1;
        }
        if (ks.key_down[GLFW_KEY_A]) {
            move_dir.x -= 1;
        }
        if (ks.key_down[GLFW_KEY_D]) {
            move_dir.x += 1;
        }
        if (vvm::dot(move_dir, move_dir) > 0)
            pworld.velocities[player_id].linear += vvm::normalize(move_dir) * 15.0f * dt;
        // else
            // pworld.velocities[player_id].linear = {0, 0};

        if (!pause || ks.key_pressed[GLFW_KEY_R]) pworld.step(1.0 / 60.0, 20);

        // for (int i = 0; i < n_bodies; ++i) {
        //     bodies[i].velocity += gravity * dt;
        //     bodies[i].position += bodies[i].velocity * dt;
        // }

        auto proj = vvm::ortho(0.1f, (float) width / height);
        // auto view = vvm::translate(vvm::v3f(-player.position, 0));

        auto viewProj = proj /* * view */;

        glViewport(0, 0, width, height);

        glClear(GL_COLOR_BUFFER_BIT);

        static const auto setUniforms = [&] (const vvm::m4f& m, const vvm::v3f color) {
            auto mvp = viewProj * m;
            // glUniformMatrix4fv(uMVP, 1, GL_FALSE, mvp.data);
            // glUniform3fv(uColor, 1, color.data);
            scene_data_buffer.write(0, 0, [&] (void* ptr) { *(vvm::m4f*) ptr = mvp; });
            program.bindUniformBuffer("scene_data", 0, scene_data_buffer, 0, scene_data_buffer.size());
            // glUniformMatrix4fv(program.getUniformLocation("mvp"), 1, GL_FALSE, mvp.data);
            glUniform3fv(program.getUniformLocation("color"), 1, color.data);
            // program.setUniform("mvp", mvp);
            // program.setUniform("color", color);
        };

        static const auto drawQuad = [&] (const vvm::v2f& size, const vvm::v2f& pos, float angle, const vvm::v3f color) {
            auto model = vvm::translate(pos) * vvm::m4f(vvm::rotate(angle)) * vvm::scale(vvm::v3f(size, 0));
            setUniforms(model, color);
            glDrawArrays(GL_LINES, 0, 10);
        };

        static const auto drawSolidQuad = [&] (const vvm::v2f& size, const vvm::v2f& pos, float angle, const vvm::v3f color) {
            auto model = vvm::translate(pos) * vvm::m4f(vvm::rotate(angle)) * vvm::scale(vvm::v3f(size, 0));
            setUniforms(model, color);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        };

        static const auto drawLine = [&] (const vvm::v2f& p0, const vvm::v2f& p1, const vvm::v3f& color) {
            auto model = vvm::translate(p0) * vvm::m4f(vvm::m2f(p1 - p0, {0, 0})) * vvm::translate(vvm::v2f{0.5, 0.5});
            setUniforms(model, color);
            glDrawArrays(GL_LINES, 0, 2);
        };
        
        for (int i = 0; i < pworld.num_bodies; ++i) {
            const auto& t = pworld.cw.transforms[i];
            const auto& s = pworld.cw.collision_shapes[pworld.cw.shape_ids[i]];
            drawQuad(2.0f * s.extents, t.position, t.angle, {1, 1, 1});
            const auto& b = pworld.cw.aabbs[i];
            if (draw_extra) {
                auto ext = b.max_extent - b.min_extent;
                auto pos = (b.max_extent + b.min_extent) * 0.5f;
                drawQuad(ext, pos, 0, {1, 0.25, 0});
            }
        }

        if (draw_extra)
        for (const auto& pair : pworld.cw.bp.pairs()) {
            const auto& b0 = pworld.cw.aabbs[pair.first];
            const auto& b1 = pworld.cw.aabbs[pair.second];
            auto bmin = vvm::min(b0.min_extent, b1.min_extent);
            auto bmax = vvm::max(b0.max_extent, b1.max_extent);
            auto ext = bmax - bmin;
            auto pos = (bmax + bmin) * 0.5f;
            drawQuad(ext, pos, 0, {1, 1, 0});
        };

        for (const auto& pair : pworld.cw.np.pairs()) {
            const auto& t0 = pworld.cw.transforms[pair.i0];
            const auto& t1 = pworld.cw.transforms[pair.i1];
            const auto& s0 = pworld.cw.collision_shapes[pworld.cw.shape_ids[pair.i0]];
            const auto& s1 = pworld.cw.collision_shapes[pworld.cw.shape_ids[pair.i1]];
            
            if (draw_extra) {
                drawLine(pair.incident_face_points[0], pair.incident_face_points[1], {0, 1, 1});
                drawLine(pair.reference_face_points[0], pair.reference_face_points[1], {0, 0, 1});
            }

            vvm::v2f p1, p2;
            if (pair.feature.b0_reference) {
                switch (pair.feature.ref_axis) {
                case physics::axis_id::x:
                    p1 = t0.position + pair.normal * s0.extents.x;
                    break;
                case physics::axis_id::y:
                    p1 = t0.position + pair.normal * s0.extents.y;
                    break;
                }
                p2 = p1 + pair.normal;
            } else {
                switch (pair.feature.ref_axis) {
                case physics::axis_id::x:
                    p1 = t1.position - pair.normal * s1.extents.x;
                    break;
                case physics::axis_id::y:
                    p1 = t1.position - pair.normal * s1.extents.y;
                    break;
                }
                p2 = p1 - pair.normal;
            }

            auto color = pair.num_contacts > 0 ? vvm::v3f(1, 0, 0) : vvm::v3f(0, 1, 0);
            drawLine(p1, p2, color);

            for (int i = 0; i < pair.num_contacts; ++i) {
                auto p = pworld.cw.np.contacts()[pair.contact_ids[i]].position;
                drawSolidQuad({0.1, 0.1}, p, 0, {1, 0, 1});
            }
        }

        glfwSwapBuffers(app.window);
    }

    // glDeleteProgram(program);
}

int main() {
    App app;

    auto err = init(app);
    
    printError(err);

    run(app);

    cleanup(err, app);

    return 0;
}