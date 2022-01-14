#pragma once

#include <vector>

#include "types.h"


namespace physics::dynamics {

struct world_data {
    std::vector<real_t> mass_inverses;
    std::vector<real_t> inertia_inverses;
    std::vector<real_t> frictions;
    std::vector<transform> transforms;
    std::vector<velocity> velocities;
    std::vector<id_t> ids;
    size_t num_bodies;
};

}