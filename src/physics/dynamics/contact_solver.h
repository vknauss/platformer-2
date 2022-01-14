#pragma once

#include <map>

#include "world_data.h"
#include "collision.h"


namespace physics::dynamics {

class contact_solver {

    struct constraint_data {
        id_t i0, i1;
        v2 point;
        real_t depth;
        m2 basis;
        v2 r0, r1;
        v2 local_impulse;
        v2 local_mass;
        real_t friction;
    };

    struct cache_key {
        id_t i0, i1;
        contact_feature feature;
    };

    struct {
        std::vector<constraint_data> constraints;
        std::map<cache_key, id_t> inds;
    } _data, _prev_data;


public:

    real_t bias = 0.2, allowed_penetration = 0.01;

    void update_constraints(const collision_world& cw);

    void pre_step(world_data& wd);

    void step(world_data& wd, real_t dt);

};

}