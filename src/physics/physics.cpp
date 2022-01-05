#include "physics.h"

#include <cmath>
#include <algorithm>
#include <iostream>

#include <vvm/matrix_tfm.hpp>

namespace physics {

bool contact_feature::operator==(const contact_feature& f) const {
    return b0_face == f.b0_face && b1_face == f.b1_face && point_side_pos == f.point_side_pos;
}

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

struct rbdata {
    real_t imass, iinertia;
    v2 pos, vel;
    real_t rot, vrot;
    real_t friction;
};

struct cdata {
    aabb bound;
    id_t shapeid;
};



struct world {
    std::vector<rbdata> bodies;

};

template<typename pair_t>
static id_t find_pair(id_t i0, id_t i1, const std::vector<pair_t>& ccs) {
    id_t pos = ccs.size() / 2;
    id_t min = 0;
    id_t end = ccs.size();
    // binary search for pair in sorted list of collision_pairs
    // the runtime of this search for all pairs is O(bpairs.size() * log2(_pairs.size()))
    // if bpairs were sorted, we could run through both arrays in linear time, but of course
    // there's the cost of sorting. maybe revisit.
    while (end - min > 0) {
        const auto& p = ccs[pos];
        if (i0 < p.i0 || (i0 == p.i0 && i1 < p.i1)) {
            // search left
            end = pos;
        } else if (i0 > p.i0 || (i0 == p.i0 && i1 > p.i1)) {
            // search right
            min = pos + 1;
        } else {
            // found
            return pos;
        }
        pos = min + (end - min) / 2;
    }
    return ID_NULL;
}

void contact_solver::update(const std::vector<collision_pair>& ps,
                            const std::vector<contact>& cs,
                            const std::vector<transform>& tfm,
                            const std::vector<real_t>& im,
                            const std::vector<real_t>& ii,
                            const std::vector<real_t>& friction) {
    // copy over prev frame data
    oldccs = ccs;
    oldsps = sps;
    ccs.resize(cs.size());
    sps.resize(ps.size());

    // sort input pairs by body ids
    auto oldsize = sort_ids.size();
    sort_ids.resize(ps.size());
    for (auto i = oldsize; i < sort_ids.size(); ++i) sort_ids[i] = i;

    std::sort(sort_ids.begin(), sort_ids.end(), [&] (auto i0, auto i1) {
        return ps[i0].i0 < ps[i1].i0 || (ps[i0].i0 == ps[i1].i0 && ps[i0].i1 < ps[i1].i1);
    });
    
    // update current frame data based on input
    auto ci = 0u;
    auto pi = 0u;
    for (auto i : sort_ids) {
        const auto& p = ps[i];
        if (im[p.i0] == 0 && im[p.i1] == 0) continue;

        auto& sp = sps[pi++];
        sp.i0 = p.i0;
        sp.i1 = p.i1;
        sp.normal = p.normal;
        sp.tangent = p.tangent;
        sp.friction = std::sqrt(friction[p.i0] * friction[p.i1]);
        sp.num_contacts = 0;
        
        // find previous match for warm starting
        auto oi = find_pair(p.i0, p.i1, oldsps);
        
        for (int j = 0; j < p.num_contacts; ++j) {
            const auto& c = cs[p.contact_ids[j]];
            
            sp.contact_ids[sp.num_contacts++] = ci;
            auto& cc = ccs[ci++];
            cc.c = c;
            cc.i0 = p.i0;
            cc.i1 = p.i1;
            cc.r0 = c.position - tfm[p.i0].position;
            cc.r1 = c.position - tfm[p.i1].position;
            cc.mass_normal = effective_mass(im[p.i0], im[p.i1], ii[p.i0], ii[p.i1], cc.r0, cc.r1, p.normal);
            cc.mass_tangent = effective_mass(im[p.i0], im[p.i1], ii[p.i0], ii[p.i1], cc.r0, cc.r1, p.tangent);
            cc.impulse_normal = 0;
            cc.impulse_tangent = 0;

            // if matching pair found, check for matching contact
            if (oi != ID_NULL) {
                const auto& osp = oldsps[oi];
                for (int k = 0; k < osp.num_contacts; ++k) {
                    const auto& occ = oldccs[osp.contact_ids[k]];
                    if (c.feature == occ.c.feature) {
                        // copy over previous accumulated impulses
                        // cc.impulse_normal = occ.impulse_normal;
                        // cc.impulse_tangent = occ.impulse_tangent;
                        break;
                    }
                }
            }
        }    
    }
    ccs.resize(ci);
    sps.resize(pi);
}

void contact_solver::solve(const std::vector<real_t>& im, const std::vector<real_t>& ii, real_t dt,
                           std::vector<velocity>& vels) {
    std::cout << "solving " << ccs.size() << " contacts" << std::endl;
    for (const auto& sp : sps) {
        auto& v0 = vels[sp.i0];
        auto& v1 = vels[sp.i1];
        auto im0 = im[sp.i0], im1 = im[sp.i1];
        auto ii0 = ii[sp.i0], ii1 = ii[sp.i1];
        for (int i = 0; i < sp.num_contacts; ++i) {
            auto& cc = ccs[sp.contact_ids[i]];
            
            // point velocities of contact point for each body
            v2 pv0 = v0.linear + vvm::cross(v0.angular, cc.r0);
            v2 pv1 = v1.linear + vvm::cross(v1.angular, cc.r1);
            
            // relative velocity at contact point
            v2 dv = pv1 - pv0;
            
            // depth penalty bias term
            real_t dd = -cc.c.depth - allowed_penetration;
            real_t vbias = dd > 0 ? dd * bias / dt : 0;

            // size of the computed impulse
            real_t impulse_normal = cc.mass_normal * (-vvm::dot(sp.normal, dv) + vbias);
        
            // accumulate impulses for better stability
            real_t acc_impulse = std::max<real_t>(cc.impulse_normal + impulse_normal, 0);
            impulse_normal = acc_impulse - cc.impulse_normal;
            cc.impulse_normal = acc_impulse;
            
            // apply impulses
            auto impulse = impulse_normal * sp.normal;
            {
                v0.linear  -= im0 * impulse;
                v1.linear  += im1 * impulse;
                v0.angular -= ii0 * vvm::cross(cc.r0, impulse);
                v1.angular += ii1 * vvm::cross(cc.r1, impulse);
            }

            pv0 = v0.linear + vvm::cross(v0.angular, cc.r0);
            pv1 = v1.linear + vvm::cross(v1.angular, cc.r1);
            dv = pv1 - pv0;
            real_t mass_tangent = effective_mass(im0, im1, ii0, ii1, cc.r0, cc.r1, sp.tangent);
            real_t impulse_tangent = mass_tangent * (-vvm::dot(sp.tangent, dv));
            auto friction_max = sp.friction * cc.impulse_normal;
            acc_impulse = vvm::clamp<real_t>(cc.impulse_tangent + impulse_tangent, -friction_max, friction_max);
            impulse_tangent = acc_impulse - cc.impulse_tangent;
            cc.impulse_tangent = acc_impulse;
            impulse = impulse_tangent * sp.tangent;
            {
                v0.linear  -= im0 * impulse;
                v1.linear  += im1 * impulse;
                v0.angular -= ii0 * vvm::cross(cc.r0, impulse);
                v1.angular += ii1 * vvm::cross(cc.r1, impulse);
            }
        }
    }
}

void dynamics_world::step(real_t dt, int iterations) {
    cw.update();
    
    // if (imasses.size() < num_bodies) {
    //     imasses.resize(num_bodies, 0);
    //     iinertias.resize(num_bodies, 0);
    //     velocities.resize(num_bodies, {.linear = {0, 0}, .angular = 0});
    // }
    // frictions.resize(num_bodies, 0.5);

    for (auto i = 0u; i < num_bodies; ++i) {
        if (imasses[i] > 0)
            velocities[i].linear += v2(0, -9.8) * dt;
    }

    const auto& contacts = cw.np.contacts();

    solver.update(cw.np.pairs(), cw.np.contacts(), cw.transforms, imasses, iinertias, frictions);

    // for ()
    //         if (auto match_id = find_matching_pair(bpairs[i], _old_pairs) != ID_NULL) {
    //             const auto& op = _old_pairs[match_id];
    //             if (op.reference_face == p.reference_face && op.incident_face == p.incident_face) {
    //                 // pair matches, keep contacts (do this later :])
    //                 std::cout << "Found matching contact pair! Delete this message after implementing warm starting." << std::endl;
    //             }
    //         }

    // for (auto& c : contacts) {
    //     c.impulse_normal = 0;
    //     c.impulse_tangent = 0;
    // }

    for (int iterc = 0; iterc < iterations; ++iterc) {
        solver.solve(imasses, iinertias, dt, velocities);
    }

    // for (int iterc = 0; iterc < iterations; ++iterc) {
    //     for (const auto& pair : pairs) {
    //         real_t sim = imasses[pair.b0] + imasses[pair.b1];
    //         if (sim == 0.0) continue;

    //         const auto& t0 = transforms[pair.b0];
    //         const auto& t1 = transforms[pair.b1];
    //         auto& v0 = velocities[pair.b0];
    //         auto& v1 = velocities[pair.b1];

    //         real_t im0 = imasses[pair.b0], im1 = imasses[pair.b1];
    //         real_t ii0 = iinertias[pair.b0], ii1 = iinertias[pair.b1];
    //         const auto& n = pair.axis;
    //         v2 t = {n.y, -n.x};

    //         real_t friction = std::sqrt(frictions[pair.b0] * frictions[pair.b1]);

    //         for (int i = 0; i < pair.num_contacts; ++i) {
    //             auto& c = cw.np.contacts()[pair.contact_ids[i]];

    //             // vectors from body centers to contact point
    //             v2 r0 = c.position - t0.position, r1 = c.position - t1.position;

    //             // point velocities of contact point for each body
    //             v2 pv0 = v0.linear + vvm::cross(v0.angular, r0);
    //             v2 pv1 = v1.linear + vvm::cross(v1.angular, r1);

    //             // relative velocity at contact point
    //             v2 dv = pv1 - pv0;

    //             // depth penalty bias term
    //             real_t dd = -c.depth - allowed_penetration;
    //             real_t vbias = dd > 0 ? dd * bias / dt : 0;
                
    //             // effective mass of the contact relative to the normal
    //             // disclaimer: im not sure how this works
    //             real_t mass_normal = effective_mass(im0, im1, ii0, ii1, r0, r1, n);

    //             // size of the computed impulse
    //             real_t impulse_normal = mass_normal * (-vvm::dot(n, dv) + vbias);
                

    //             // accumulate impulses for better stability
    //             real_t acc_impulse = std::max<real_t>(c.impulse_normal + impulse_normal, 0);
    //             impulse_normal = acc_impulse - c.impulse_normal;
    //             c.impulse_normal = acc_impulse;


    //             // apply impulses
    //             auto impulse = impulse_normal * n;
    //             {
    //                 v0.linear  -= im0 * impulse;
    //                 v1.linear  += im1 * impulse;
    //                 v0.angular -= ii0 * vvm::cross(r0, impulse);
    //                 v1.angular += ii1 * vvm::cross(r1, impulse);
    //             }

    //             pv0 = v0.linear + vvm::cross(v0.angular, r0);
    //             pv1 = v1.linear + vvm::cross(v1.angular, r1);
    //             dv = pv1 - pv0;
    //             real_t mass_tangent = effective_mass(im0, im1, ii0, ii1, r0, r1, t);
    //             real_t impulse_tangent = mass_tangent * (-vvm::dot(t, dv));
    //             auto friction_max = friction * c.impulse_normal;
    //             acc_impulse = vvm::clamp<real_t>(c.impulse_tangent + impulse_tangent, -friction_max, friction_max);
    //             impulse_tangent = acc_impulse - c.impulse_tangent;
    //             c.impulse_tangent = acc_impulse;
    //             impulse = impulse_tangent * t;
    //             {
    //                 v0.linear  -= im0 * impulse;
    //                 v1.linear  += im1 * impulse;
    //                 v0.angular -= ii0 * vvm::cross(r0, impulse);
    //                 v1.angular += ii1 * vvm::cross(r1, impulse);
    //             }
    //         }
    //     }
    // }
    
    for (auto i = 0u; i < num_bodies; ++i) {
        cw.transforms[i].position += velocities[i].linear * dt;
        cw.transforms[i].angle += velocities[i].angular * dt;
    }
}

id_t dynamics_world::add_collision_shape(const collision_shape &shape) {
    return cw.add_collision_shape(shape);
}

id_t dynamics_world::add_rigid_body(real_t mass, const transform& tfm, id_t shape_id, real_t friction) {
    auto id = cw.add_collision_object(tfm, shape_id);
    ++num_bodies;
    if (cw.num_bodies != num_bodies) std::cerr << "!" << std::endl;
    
    real_t imass = mass > 0 ? 1.0 / mass : 0.0;
    const auto& e = cw.collision_shapes[cw.shape_ids[id]].extents;
    auto dd = vvm::dot(e, e);
    real_t iinertia = dd > 0 ? 3.0 * imass / dd : 0.0;
    
    imasses.push_back(imass);
    iinertias.push_back(iinertia);
    velocities.push_back({.linear = {0, 0}, .angular = 0});
    frictions.push_back(friction);
    
    return id;
}

dynamics_world::dynamics_world() : num_bodies(0) { }
dynamics_world::~dynamics_world() { }

};  // namespace physics