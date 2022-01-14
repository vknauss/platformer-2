#pragma once

#include "world_data.h"
#include "contact_solver.h"
#include "collision.h"

namespace physics::dynamics {


struct rigid_body {
    transform tfm;
    velocity vel;
    real_t mass, inertia, friction;
    id_t shape_id;
};

class world {
public:

    real_t timestep = 1.0 / 60.0;
    int iterations = 5;

    void update(real_t dt);

    void apply_impulse(id_t body_id, const v2& impulse, const v2& offset);

    id_t add_rigid_body(const rigid_body& body);

    template<typename collision_shape_t>
    id_t add_collision_shape(const collision_shape_t& shape);

    rigid_body get_rigid_body(id_t body_id) const;

    template<typename collision_shape_t>
    collision_shape_t get_collision_shape(id_t shape_id) const;


private:

    world_data _data;

    contact_solver _solver;

    collision_world _collision_world;

    real_t _interp;

    void step();

};

template<typename collision_shape_t>
id_t world::add_collision_shape(const collision_shape_t& shape) {
    return _collision_world.add_collision_shape(shape);
}

template<typename collision_shape_t>
collision_shape_t world::get_collision_shape(id_t shape_id) const {
    return _collision_world.get_collision_shape<collision_shape_t>(shape_id);
}


}