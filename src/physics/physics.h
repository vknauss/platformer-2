#pragma once

#include <vvm/vvm.hpp>

namespace physics {

typedef float real_t;

using v2 = vvm::v2<real_t>;
using m2 = vvm::m2<real_t>;

struct collision_shape {
    v2 extents;
};

struct body_state {
    v2 position;
    real_t angle;

    v2 velocity;
    real_t ang_vel;
};

struct rigid_body {
    const real_t mass;
    const collision_shape* shape;

    body_state state;
};

struct aabb {
    v2 min_extent, max_extent;
};

constexpr aabb get_shape_aabb(const collision_shape& shape, const v2& position, real_t angle);

constexpr bool aabbs_intersect(const aabb& b0, const aabb& b1);

struct contact_point {
    v2 position;
    v2 normal;
    float separation;
};

struct contact_manifold {
    rigid_body* b0, * b1;
    int num_points = 0;
    contact_point* points = nullptr;
};

contact_manifold compute_contact(rigid_body* b0, rigid_body* b1);


};  // namespace physics