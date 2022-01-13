#pragma once

#include "types.h"


namespace physics {

struct rigid_body {
    transform tfm;
    velocity vel;
    real_t mass, inertia, friction;
    id_t shape_id;
};

class collision_shape;

class dynamics_world {
public:

    dynamics_world();
    ~dynamics_world();

    void update(real_t dt);

    id_t add_collision_shape(const collision_shape& shape);

    id_t add_rigid_body(const rigid_body& body);

private:

    class impl;
    impl* _impl;

};

}