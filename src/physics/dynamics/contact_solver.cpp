#include "contact_solver.h"


namespace physics::dynamics {

void contact_solver::update_constraints(const collision_world& cw) {
    _prev_data = _data;

    _data.constraints.reserve(cw.get_contacts().size());
    _data.constraints.clear();
    _data.inds.clear();

    for (const auto& pair : cw.get_pairs()) {
        for (int pci = 0; pci < pair.num_contacts; ++pci) {
            const auto& contact = cw.get_contacts()[pair.contact_ids[pci]];
            
            constraint_data constraint {pair.i0, pair.i1, contact.position,
                contact.depth, m2(pair.tangent, pair.normal)};
            
            cache_key k {pair.i0, pair.i1, contact.feature};
            if (auto it = _prev_data.inds.find(k); it != _prev_data.inds.end()) {
                constraint.local_impulse = _prev_data.constraints[it->second].local_impulse;
            } else {
                constraint.local_impulse = v2(0);
            }

            _data.inds.insert({k, _data.constraints.size()});
            _data.constraints.push_back(constraint);
        }
    }
}

static constexpr real_t effective_mass(real_t im0, real_t im1, real_t ii0, real_t ii1, const v2& r0, const v2& r1, const v2& n) {
    auto r0xn = vvm::cross(r0, n), r1xn = vvm::cross(r1, n);
    real_t denom = im0 + im1 + vvm::dot(n, ii0 * vvm::cross(r0xn, r0) + ii1 * vvm::cross(r1xn, r1));
    return 1.0 / denom;
}

static constexpr v2 compute_local_mass(real_t im0, real_t im1, real_t ii0, real_t ii1, const v2& r0, const v2& r1, const m2& basis) {
    return v2(effective_mass(im0, im1, ii0, ii1, r0, r1, basis[0]), effective_mass(im0, im1, ii0, ii1, r0, r1, basis[1]));
}

void contact_solver::pre_step(world_data& wd) {
    for (auto& c : _data.constraints) {
        c.r0 = c.point - wd.transforms[c.i0].position;
        c.r1 = c.point - wd.transforms[c.i1].position;
        c.local_mass = compute_local_mass(wd.mass_inverses[c.i0], wd.mass_inverses[c.i1],
            wd.inertia_inverses[c.i0], wd.inertia_inverses[c.i1], c.r0, c.r1, c.basis);
        c.friction = std::sqrt(wd.frictions[c.i0] * wd.frictions[c.i1]);

        // warm starting
        auto impulse = c.basis * c.local_impulse;
        wd.velocities[c.i0].linear -= wd.mass_inverses[c.i0] * impulse;
        wd.velocities[c.i1].linear += wd.mass_inverses[c.i1] * impulse;
        wd.velocities[c.i0].angular -= wd.inertia_inverses[c.i0] * vvm::cross(c.r0, impulse);
        wd.velocities[c.i1].angular += wd.inertia_inverses[c.i1] * vvm::cross(c.r1, impulse);
    }
}

static constexpr v2 point_velocity(const velocity& v, const v2& r) {
    return v.linear + vvm::cross(v.angular, r);
}

void contact_solver::step(world_data& wd, real_t dt) {
    for (auto& c : _data.constraints) {
        // relative velocity at contact point
        auto dv = point_velocity(wd.velocities[c.i1], c.r1);
        dv -= point_velocity(wd.velocities[c.i0], c.r0);

        // depth penalty bias term
        real_t dd = -c.depth - allowed_penetration;
        real_t vbias = dd > 0 ? dd * bias / dt : 0;

        // size of the computed impulse
        real_t impulse_normal = c.local_mass[1] * (-vvm::dot(c.basis[1], dv) + vbias);
    
        // accumulate impulses for better stability
        real_t acc_impulse = std::max<real_t>(c.local_impulse[1] + impulse_normal, 0);
        impulse_normal = acc_impulse - c.local_impulse[1];
        c.local_impulse[1] = acc_impulse;
        
        // apply impulses
        auto impulse = impulse_normal * c.basis[1];
        wd.velocities[c.i0].linear  -= wd.mass_inverses[c.i0] * impulse;
        wd.velocities[c.i1].linear  += wd.mass_inverses[c.i1] * impulse;
        wd.velocities[c.i0].angular -= wd.inertia_inverses[c.i0] * vvm::cross(c.r0, impulse);
        wd.velocities[c.i1].angular -= wd.inertia_inverses[c.i1] * vvm::cross(c.r1, impulse);
        
        // recompute dv based on applied normal impulse
        dv = point_velocity(wd.velocities[c.i1], c.r1);
        dv -= point_velocity(wd.velocities[c.i0], c.r0);

        // compute tangent impulse
        real_t impulse_tangent = c.local_mass[0] * (-vvm::dot(c.basis[0], dv));
        
        // accumulate tangent impulse
        auto friction_max = c.friction * c.local_impulse[1];
        acc_impulse = vvm::clamp<real_t>(c.local_impulse[0] + impulse_tangent, -friction_max, friction_max);
        impulse_tangent = acc_impulse - c.local_impulse[0];
        c.local_impulse[0] = acc_impulse;

        // apply tangent impulse
        impulse = impulse_tangent * c.basis[0];
        wd.velocities[c.i0].linear  -= wd.mass_inverses[c.i0] * impulse;
        wd.velocities[c.i1].linear  += wd.mass_inverses[c.i1] * impulse;
        wd.velocities[c.i0].angular -= wd.inertia_inverses[c.i0] * vvm::cross(c.r0, impulse);
        wd.velocities[c.i1].angular -= wd.inertia_inverses[c.i1] * vvm::cross(c.r1, impulse);
    }
}

}