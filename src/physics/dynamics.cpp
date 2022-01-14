#include "dynamics.h"

#include <vector>

#include "collision.h"


namespace physics {

class dynamics_world::impl {
public:

    // more body data
    std::vector<velocity> velocities;
    std::vector<real_t> imasses;
    std::vector<real_t> iinertias;
    std::vector<real_t> frictions;

    std::vector<transform> prev_tfms;
    std::vector<transform> interp_tfms;

    collision_world cw;

    contact_solver solver;

    size_t num_bodies;

    real_t interp;
    
    template<typename collision_shape_t>
    id_t add_collision_shape(const collision_shape_t& shape) {
        return cw.add_collision_shape(shape);
    }

    id_t add_rigid_body(const rigid_body& body) {
        collision_object obj {};
        obj.tfm = body.tfm;
        obj.shape_id = body.shape_id;
        auto id = cw.add_collision_object(obj);

        

        return id;
    }

    void apply_impulse(id_t body_id, const v2& impulse, const v2& offset);

    void step(real_t dt, int iterations);

    void update(real_t dt);

};







void dynamics_world::update(real_t dt) {
    return _impl->update(dt);
}

template<typename collision_shape_t>
id_t dynamics_world::add_collision_shape(const collision_shape_t& shape) {
    return _impl->add_collision_shape(shape);
}

id_t dynamics_world::add_rigid_body(const rigid_body& body) {
    return _impl->add_rigid_body(body);
}



dynamics_world::dynamics_world() {
    _impl = new impl;
}

dynamics_world::~dynamics_world() {
    delete _impl;
}

}