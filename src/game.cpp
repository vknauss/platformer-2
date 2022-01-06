#include "game.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <vvm/matrix_tfm.hpp>

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
    player_id = pw.add_rigid_body(1, {.position = {0, 0}, .angle = 0}, box_shape_id, 0.5);

    // positive mass rigid bodies are dynamic
    pw.add_rigid_body(1, {.position = {5, 0}}, box_shape_id, 0.5);
    pw.add_rigid_body(1, {.position = {5, 2.5}}, box_shape_id, 0.5);
    pw.add_rigid_body(1, {.position = {5, 5.0}}, box_shape_id, 0.5);
    pw.add_rigid_body(1, {.position = {5, 8.0}}, box_shape_id, 0.5);
    pw.add_rigid_body(1, {.position = {5, 11.0}}, box_shape_id, 0.5);
    pw.add_rigid_body(1, {.position = {5, 14.0}}, box_shape_id, 0.5);
    
    // 0 mass bodies are static
    pw.add_rigid_body(0, {.position = {0, -4}, .angle = 0.0}, ground_shape_id, 0.5);

    // add a table
    pw.add_rigid_body(0.1, {.position = {-1.4, -2.5}}, table_leg_shape_id, 0.5);
    pw.add_rigid_body(0.1, {.position = { 1.4, -2.5}}, table_leg_shape_id, 0.5);
    pw.add_rigid_body(0.1, {.position = { 0.0, -1.95}}, table_shape_id, 0.5);
}

game::status test_game::update(float dt) {
    if (ks.key_down[GLFW_KEY_ESCAPE]) return game::status::quit;

    if (ks.key_pressed[GLFW_KEY_P]) pause = !pause;

    if (ks.key_pressed[GLFW_KEY_E]) draw_extra = !draw_extra;
    
    ks.key_pressed.clear();

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
        pw.velocities[player_id].linear += vvm::normalize(move_dir) * 15.0f * dt;

    if (!pause || ks.key_pressed[GLFW_KEY_R]) pw.step(1.0 / 60.0, 20);

    return game::status::running;
}

void test_game::render(uint32_t width, uint32_t height) {
    auto proj = vvm::ortho(0.1f, (float) width / height);

    r.set_camera_matrix(proj);
    
    for (int i = 0; i < pw.num_bodies; ++i) {
        const auto& t = pw.cw.transforms[i];
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
        r.draw_filled_quad(ext, pos, 0, {1, 1, 0});
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
}
