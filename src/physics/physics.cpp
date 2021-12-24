#include "physics.h"

#include <cmath>
#include <iostream>

#include <vvm/matrix_tfm.hpp>

namespace physics {

void dynamics_world::step(real_t dt, int iterations) {
    find_intersecting_pairs();
    find_contacts();
    
    if (imasses.size() < num_bodies) {
        imasses.resize(num_bodies, 0);
        iinertias.resize(num_bodies, 0);
        velocities.resize(num_bodies, {.linear = {0, 0}, .angular = 0});
    }

    for (auto i = 0u; i < num_bodies; ++i) {
        if (imasses[i] > 0)
            velocities[i].linear += v2(0, -9.8) * dt;
    }

    for (int iterc = 0; iterc < iterations; ++iterc) {
        for (const auto& pair : pairs) {
            real_t sim = imasses[pair.b0] + imasses[pair.b1];
            if (sim == 0.0) continue;

            const auto& t0 = transforms[pair.b0];
            const auto& t1 = transforms[pair.b1];
            auto& v0 = velocities[pair.b0];
            auto& v1 = velocities[pair.b1];

            real_t im0 = imasses[pair.b0], im1 = imasses[pair.b1];
            real_t ii0 = iinertias[pair.b0], ii1 = iinertias[pair.b1];
            const auto& n = pair.axis;

            for (int i = 0; i < pair.num_contacts; ++i) {
                const auto& c = contacts[pair.contact_ids[i]];

                v2 r0 = c.position - t0.position, r1 = c.position - t1.position;
                auto r0xn = vvm::cross(r0, n), r1xn = vvm::cross(r1, n);

                v2 pv0 = v0.linear + vvm::cross(v0.angular, r0);
                v2 pv1 = v1.linear + vvm::cross(v1.angular, r1);

                v2 dv = pv1 - pv0;
                real_t denom = sim + vvm::dot(n, ii0 * vvm::cross(r0xn, r0) + ii1 * vvm::cross(r1xn, r1));

                real_t impulse = -vvm::dot(n, dv) / denom;
                if (impulse > 0) {
                    v0.linear -= im0 * impulse * n;
                    v0.angular -= ii0 * impulse * r0xn;
                    v1.linear += im1 * impulse * n;
                    v1.angular += ii1 * impulse * r1xn;
                }
            }
        }
    }
    
    for (auto i = 0u; i < num_bodies; ++i) {
        transforms[i].position += velocities[i].linear * dt;
        transforms[i].angle += velocities[i].angular * dt;
    }
}

id_t dynamics_world::add_rigid_body(real_t mass, const transform& tfm, id_t shape_id) {
    auto id = add_collision_object(tfm, shape_id);
    real_t imass = mass > 0 ? 1.0 / mass : 0.0;
    const auto& e = collision_shapes[shape_ids[id]].extents;
    auto dd = vvm::dot(e, e);
    real_t iinertia = dd > 0 ? 3.0 * imass / dd : 0.0;
    if (imasses.size() < id) {
        imasses.resize(num_bodies, 0);
        iinertias.resize(num_bodies, 0);
        velocities.resize(num_bodies, {.linear = {0, 0}, .angular = 0});
        imasses[id] = imass;
        iinertias[id] = iinertia;
    } else {
        imasses.push_back(imass);
        iinertias.push_back(iinertia);
        velocities.push_back({.linear = {0, 0}, .angular = 0});
    }
    return id;
}

dynamics_world::dynamics_world() { }
dynamics_world::~dynamics_world() { }

};  // namespace physics