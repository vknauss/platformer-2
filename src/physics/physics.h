#pragma once

#include <vvm/vvm.hpp>

#include <cstdint>
#include <limits>
#include <map>
#include <vector>

#include "types.h"


namespace physics {









class dynamics_world {

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

public:
    
    id_t add_collision_shape(const collision_shape& shape);
    id_t add_rigid_body(const rigid_body& body);

    void apply_impulse(id_t body_id, const v2& impulse, const v2& offset);

    void step(real_t dt, int iterations);

    void update(real_t dt, int iterations);

    rigid_body get_rigid_body(id_t id) const;

    // const collision_shape& get_collision_shape(id_t id) const {
    //     return cw.collision_shapes[id];
    // }

    const collision_world& get_collision_world() const {
        return cw;
    }

    dynamics_world();
    virtual ~dynamics_world();

};

inline const std::vector<broadphase::pair>& broadphase::pairs() const {
    return _pairs;
}

inline const std::vector<collision_pair>& narrowphase::pairs() const {
    return _pairs;
}

inline const std::vector<contact>& narrowphase::contacts() const {
    return _contacts;
}

};  // namespace physics