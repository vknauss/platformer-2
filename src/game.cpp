#include "game.h"

#include <iostream>
#include <map>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <vvm/matrix_tfm.hpp>
#include <vvm/string.hpp>

#include "ecs.h"
#include "physics/collision.h"
#include "physics/dynamics.h"
#include "render/render.h"


struct keyboard_state {
    std::map<int, bool> key_down;
    std::map<int, bool> key_pressed;
};

template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
T random() {
    return static_cast<T>(rand()) / static_cast<T>(RAND_MAX);
}

float randf() { return random<float>(); }

class test_game : public game {

    GLFWwindow* w;

    render::renderer r;

    physics::dynamics_world pw;

    physics::id_t box_shape_id;

    bool pause = false;
    bool draw_extra = false;

    ecs::entity player;

    ecs::entity_manager em;
    ecs::packed_array<physics::id_t, 100000, 100000> rb_ids;

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
    box_shape_id = pw.add_collision_shape(physics::box_collision_shape{.extents = {1, 1}});
    auto ground_shape_id = pw.add_collision_shape(physics::box_collision_shape{.extents = {10, 1}});

    // 0 mass rigid bodies are static / kinematic: 1-way interaction with dynamic bodies
    player = em.create();
    auto player_rb_id = pw.add_rigid_body(create_rigid_body(1.0, {0, 0}, 0, box_shape_id));
    rb_ids.insert(player, player_rb_id);

    // positive mass rigid bodies are dynamic
    for (auto i = 0; i < 5; ++i) {
        auto box = em.create();
        auto box_rb_id = pw.add_rigid_body(create_rigid_body(1, {5, 2.5f * i}, 0, box_shape_id));
        rb_ids.insert(box, box_rb_id);
    }
    
    // 0 mass bodies are static
    auto ground = em.create();
    auto ground_rb_id = pw.add_rigid_body(create_rigid_body(0.0, {0, -4}, 0, ground_shape_id));
    rb_ids.insert(ground, ground_rb_id);

    // // add a table
    // auto table_leg1 = em.create();
    // auto table_leg1_rb_id = pw.add_rigid_body(create_rigid_body(1.0, {-1.4, -2.5}, 0, table_leg_shape_id));
    // rb_ids.insert(table_leg1, table_leg1_rb_id);

    // auto table_leg2 = em.create();
    // auto table_leg2_rb_id = pw.add_rigid_body(create_rigid_body(1.0, { 1.4, -2.5}, 0, table_leg_shape_id));
    // rb_ids.insert(table_leg2, table_leg2_rb_id);
    
    // auto table_top = em.create();
    // auto table_top_rb_id = pw.add_rigid_body(create_rigid_body(1.0, { 0.0, -1.95}, 0, table_shape_id));
    // rb_ids.insert(table_top, table_top_rb_id);
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
        pw.apply_impulse(rb_ids.get(player), vvm::normalize(move_dir) * 15.0f * dt, {0, 0});

    // if (!pause || ks.key_pressed[GLFW_KEY_R]) pw.step(1.0 / 60.0, 20);
    if (!pause) pw.update(dt);
    // else if (ks.key_pressed[GLFW_KEY_R]) pw.step(1.0 / 60.0, 5);

    if (ks.key_pressed[GLFW_KEY_G]) {
        auto rb = create_rigid_body(1, {0, 0}, randf(), box_shape_id);
        auto vela = randf();
        rb.vel.linear = 10.f * randf() * vvm::rotate(vela)[0];
        auto e = em.create();
        auto rbi = pw.add_rigid_body(rb);
        rb_ids.insert(e, rbi);
    }

    ks.key_pressed.clear();

    return game::status::running;
}

void test_game::render(uint32_t width, uint32_t height) {
    auto proj = vvm::ortho(0.1f, (float) width / height);

    r.set_camera_matrix(proj);
    
    em.for_each([this] (ecs::entity e) {
        auto b = pw.get_rigid_body(rb_ids.get(e));
        auto s = pw.get_collision_shape<physics::box_collision_shape>(b.shape_id);
        r.draw_quad(2.f * s.extents, b.tfm.position, b.tfm.angle, {1, 1, 1});
    });

    // if (draw_extra) {
    //     const auto& cw = pw.get_collision_world();

    //     for (const auto& pair : cw.get_narrowphase().pairs()) {
    //         auto b0 = pw.get_rigid_body(pair.i0), b1 = pw.get_rigid_body(pair.i1);
    //         auto s0 = pw.get_collision_shape(b0.shape_id), s1 = pw.get_collision_shape(b1.shape_id);
        
        
    //         r.draw_line(pair.incident_face_points[0], pair.incident_face_points[1], {0, 1, 1});
    //         r.draw_line(pair.reference_face_points[0], pair.reference_face_points[1], {0, 0, 1});
            

    //         vvm::v2f p1, p2;
    //         if (pair.feature.b0_reference) {
    //             switch (pair.feature.ref_axis) {
    //             case physics::axis_id::x:
    //                 p1 = b0.tfm.position + pair.normal * s0.extents.x;
    //                 break;
    //             case physics::axis_id::y:
    //                 p1 = b0.tfm.position + pair.normal * s0.extents.y;
    //                 break;
    //             }
    //             p2 = p1 + pair.normal;
    //         } else {
    //             switch (pair.feature.ref_axis) {
    //             case physics::axis_id::x:
    //                 p1 = b1.tfm.position - pair.normal * s1.extents.x;
    //                 break;
    //             case physics::axis_id::y:
    //                 p1 = b1.tfm.position - pair.normal * s1.extents.y;
    //                 break;
    //             }
    //             p2 = p1 - pair.normal;
    //         }

    //         auto color = pair.num_contacts > 0 ? vvm::v3f(1, 0, 0) : vvm::v3f(0, 1, 0);
    //         r.draw_line(p1, p2, color);

    //         for (int i = 0; i < pair.num_contacts; ++i) {
    //             auto p = cw.get_narrowphase().contacts()[pair.contact_ids[i]].position;
    //             r.draw_filled_quad({0.1, 0.1}, p, 0, {1, 0, 1});
    //         }
    //     }
    // }
    

    

    r.render();
}
