#pragma once

#include "types.h"
#include "collision_shape.h"


namespace physics {

struct rigid_body {
    transform tfm;
    velocity vel;
    real_t mass, inertia, friction;
    id_t shape_id;
};

class dynamics_world {
public:

    dynamics_world();
    ~dynamics_world();

    void update(real_t dt);

    template<typename collision_shape_t>
    id_t add_collision_shape(const collision_shape_t& shape);

    id_t add_rigid_body(const rigid_body& body);

    void apply_impulse(id_t body_id, const v2& impulse, const v2& offset);

    rigid_body get_rigid_body(id_t body_id) const;

    template<typename collision_shape_t>
    collision_shape_t get_collision_shape(id_t shape_id) const;

private:

    class impl;
    impl* _impl;

};

}