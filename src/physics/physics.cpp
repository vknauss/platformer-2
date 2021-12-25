#include "physics.h"

#include <cmath>
#include <iostream>

#include <vvm/matrix_tfm.hpp>

namespace physics {

// im: inverse mass, ii: inverse inertia, r: vector to point from center of mass, n: direction
// let's try to reverse engineer and figure this out
// im0 = 1/m0, im1 = 1/m1, ii0 = 1/i0, ii1 = 1/i1
// r0 x n = r0x * ny - r0y * nx, r1 x n = r1x * ny - r1y * nx
// (r0 x n) x r0 = (r0 x n) * {-r0y, r0x}
// = { -r0y * (r0x * ny - r0y * nx), r0x * (r0x * ny - r0y * nx) }
// = { r0y^2 * nx - r0x * r0y * ny , r0x^2 * ny - r0x * r0y * nx }
// (r1 x n) x r1 =
//   { r1y^2 * nx - r1x * r1y * ny , r1x^2 * ny - r1x * r1y * nx }
// ii0 ((r0 x n) x r0) + ii1 ((r1 x n) x r1) =
//   { nx (ii0 * r0y^2 + ii1 * r1y^2) - ny (ii0 * r0x * r0y + ii1 * r1x * r1y),
//     ny (ii0 * r0x^2 + ii1 * r1x^2) - nx (ii0 * r0x * r0y + ii1 * r1x * r1y) }
// (ii0 ((r0 x n) x r0) + ii1 ((r1 x n) x r1)) * n =
//   nx^2 (ii0 * r0y^2 + ii1 * r1y^2) - nx * ny (ii0 * r0x * r0y + ii1 * r1x * r1y) +
//   ny^2 (ii0 * r0x^2 + ii1 * r1x^2) - nx * ny (ii0 * r0x * r0y + ii1 * r1x * r1y)
// = nx^2 (ii0 * r0y^2 + ii1 * r1y^2) - 2 * nx * ny (ii0 * r0x * r0y + ii1 * r1x * r1y) + ny^2 (ii0 * r0x^2 + ii1 * r1x^2)
// = ii0 (nx^2 * r0y^2 + ny^2 * r0x^2 - 2 * nx * ny * r0x * r0y) + ii1 (nx^2 * r1y^2 + ny^2 * r1x^2 - 2 * nx * ny * r1x * r1y)
// aside: nx^2 * ry^2 + ny^2 * rx^2 - 2 * nx * ny * rx * ry
//        = (nx * ry)^2 + (ny * rx)^2 - 2(nx * ry)(ny * rx)
//        = (nx * ry - ny * rx)^2
// = ii0 (nx * r0y - ny * r0x)^2 + ii1 (nx * r1y - ny * r1x)^2
//   let t = {-ny, nx}
// = ii0 (r0 * t)^2 + ii1 (r1 * t)^2
// yeah... not getting this any better than i was
// at least i seem to have stumbled on an alternate formulation...
static constexpr real_t effective_mass(real_t im0, real_t im1, real_t ii0, real_t ii1, const v2& r0, const v2& r1, const v2& n) {
    auto r0xn = vvm::cross(r0, n), r1xn = vvm::cross(r1, n);
    real_t denom = im0 + im1 + vvm::dot(n, ii0 * vvm::cross(r0xn, r0) + ii1 * vvm::cross(r1xn, r1));
    return 1.0 / denom;
}

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

    for (auto& c : contacts) {
        c.impulse = 0;
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
                auto& c = contacts[pair.contact_ids[i]];

                // vectors from body centers to contact point
                v2 r0 = c.position - t0.position, r1 = c.position - t1.position;

                // point velocities of contact point for each body
                v2 pv0 = v0.linear + vvm::cross(v0.angular, r0);
                v2 pv1 = v1.linear + vvm::cross(v1.angular, r1);

                // relative velocity at contact point
                v2 dv = pv1 - pv0;

                // depth penalty bias term
                real_t dd = -c.depth - allowed_penetration;
                real_t vbias = dd > 0 ? dd * bias / dt : 0;
                
                // effective mass of the contact relative to the normal
                // disclaimer: im not sure how this works
                real_t mass_normal = effective_mass(im0, im1, ii0, ii1, r0, r1, n);

                // size of the computed impulse
                real_t impulse_normal = mass_normal * (-vvm::dot(n, dv) + vbias);

                // accumulate impulses for better stability
                real_t acc_impulse = std::max<real_t>(c.impulse + impulse_normal, 0);
                impulse_normal = acc_impulse - c.impulse;
                c.impulse = acc_impulse;

                // apply impulses
                {
                    v0.linear  -= im0 * impulse_normal * n;
                    v1.linear  += im1 * impulse_normal * n;
                    v0.angular -= ii0 * impulse_normal * vvm::cross(r0, n);
                    v1.angular += ii1 * impulse_normal * vvm::cross(r1, n);
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