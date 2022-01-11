#include "game.h"

#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <vvm/matrix_tfm.hpp>
#include <vvm/string.hpp>

#include "physics/physics.h"
#include "render/render.h"


struct keyboard_state {
    std::map<int, bool> key_down;
    std::map<int, bool> key_pressed;
};

class test_game : public game {

    GLFWwindow* w;

    render::renderer r;

    physics::dynamics_world pw;

    bool pause = false;
    bool draw_extra = false;

    physics::id_t player_id;

public:

    keyboard_state ks;

    void init(GLFWwindow* window) override;

    status update(float dt) override;

    void render(uint32_t width, uint32_t height) override;

};

game* game::create() {
    return new test_game;
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    auto g = (test_game*) glfwGetWindowUserPointer(window);
    if (action == GLFW_PRESS) {
        g->ks.key_down[key] = true;
        g->ks.key_pressed[key] = true;
    } else if (action == GLFW_RELEASE) {
        g->ks.key_down[key] = false;
    }
}

static constexpr physics::rigid_body create_rigid_body(float mass, const vvm::v2f& pos, float angle, physics::id_t shape_id) {
    physics::rigid_body result {};
    result.mass = mass;
    result.tfm.position = pos;
    result.tfm.angle = angle;
    result.shape_id = shape_id;
    result.friction = 0.5;
    return result;
}

void test_game::init(GLFWwindow* window) {
    w = window;

    glfwSetWindowUserPointer(w, this);

    glfwSetKeyCallback(w, key_callback);

    // create some collision shapes
    auto box_shape_id = pw.add_collision_shape({.extents = {0.5, 0.5}});
    auto ground_shape_id = pw.add_collision_shape({.extents = {10, 1}});
    auto table_shape_id = pw.add_collision_shape({.extents = {1.5, 0.05}});
    auto table_leg_shape_id = pw.add_collision_shape({.extents = {0.15, 0.5}});

    // 0 mass rigid bodies are static / kinematic: 1-way interaction with dynamic bodies
    player_id = pw.add_rigid_body(create_rigid_body(1.0, {0, 0}, 0, box_shape_id));

    // positive mass rigid bodies are dynamic
    for (auto i = 0; i < 5; ++i) {
        pw.add_rigid_body(create_rigid_body(1, {5, 2.5f * i}, 0, box_shape_id));
    }
    
    // 0 mass bodies are static
    pw.add_rigid_body(create_rigid_body(0.0, {0, -4}, 0, ground_shape_id));

    // add a table
    player_id = pw.add_rigid_body(create_rigid_body(1.0, {-1.4, -2.5}, 0, table_leg_shape_id));
    player_id = pw.add_rigid_body(create_rigid_body(1.0, { 1.4, -2.5}, 0, table_leg_shape_id));
    player_id = pw.add_rigid_body(create_rigid_body(1.0, { 0.0, -1.95}, 0, table_shape_id));
}

game::status test_game::update(float dt) {
    if (ks.key_down[GLFW_KEY_ESCAPE]) return game::status::quit;

    if (ks.key_pressed[GLFW_KEY_P]) pause = !pause;

    if (ks.key_pressed[GLFW_KEY_E]) draw_extra = !draw_extra;
    
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
        pw.apply_impulse(player_id, vvm::normalize(move_dir) * 15.0f * dt, {0, 0});

    // if (!pause || ks.key_pressed[GLFW_KEY_R]) pw.step(1.0 / 60.0, 20);
    if (!pause) pw.update(dt, 5);
    else if (ks.key_pressed[GLFW_KEY_R]) pw.step(1.0 / 60.0, 5);

    ks.key_pressed.clear();

    return game::status::running;
}

void test_game::render(uint32_t width, uint32_t height) {
    auto proj = vvm::ortho(0.1f, (float) width / height);

    // std::cout << vvm::to_string(proj) << std::endl;

    r.set_camera_matrix(proj);
    
    for (auto i = 0u; i < pw.num_bodies; ++i) {
        auto rb = pw.get_rigid_body(i)
        const auto& t = pw.get_transform(i);
        const auto& s = pw.cw.collision_shapes[pw.cw.shape_ids[i]];
        r.draw_quad(2.0f * s.extents, t.position, t.angle, {1, 1, 1});
        const auto& b = pw.cw.aabbs[i];
        if (draw_extra) {
            auto ext = b.max_extent - b.min_extent;
            auto pos = (b.max_extent + b.min_extent) * 0.5f;
            r.draw_quad(ext, pos, 0, {1, 0.25, 0});
        }
    }

    if (draw_extra)
    for (const auto& pair : pw.cw.bp.pairs()) {
        const auto& b0 = pw.cw.aabbs[pair.first];
        const auto& b1 = pw.cw.aabbs[pair.second];
        auto bmin = vvm::min(b0.min_extent, b1.min_extent);
        auto bmax = vvm::max(b0.max_extent, b1.max_extent);
        auto ext = bmax - bmin;
        auto pos = (bmax + bmin) * 0.5f;
        r.draw_quad(ext, pos, 0, {1, 1, 0});
    };

    for (const auto& pair : pw.cw.np.pairs()) {
        const auto& t0 = pw.cw.transforms[pair.i0];
        const auto& t1 = pw.cw.transforms[pair.i1];
        const auto& s0 = pw.cw.collision_shapes[pw.cw.shape_ids[pair.i0]];
        const auto& s1 = pw.cw.collision_shapes[pw.cw.shape_ids[pair.i1]];
        
        if (draw_extra) {
            r.draw_line(pair.incident_face_points[0], pair.incident_face_points[1], {0, 1, 1});
            r.draw_line(pair.reference_face_points[0], pair.reference_face_points[1], {0, 0, 1});
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
        r.draw_line(p1, p2, color);

        for (int i = 0; i < pair.num_contacts; ++i) {
            auto p = pw.cw.np.contacts()[pair.contact_ids[i]].position;
            r.draw_filled_quad({0.1, 0.1}, p, 0, {1, 0, 1});
        }
    }

    r.render();
}
