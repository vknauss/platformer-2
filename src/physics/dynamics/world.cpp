#pragma once

#include "world.h"


namespace physics::dynamics {

void world::step() {
    _collision_world.update();

    for (auto i = 0u; i < _data.num_bodies; ++i) {
        if (_data.mass_inverses[i] > 0) {
            _data.velocities[i].linear += v2(0, -9.8) * timestep;
        }
    }

    _solver.update_constraints(_collision_world);
    _solver.pre_step(_data);

    for (int i = 0; i < iterations; ++i) {
        _solver.step(_data, timestep);
    }
}

}