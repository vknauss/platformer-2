#pragma once

#include <vvm/vvm.hpp>
#include <vector>

namespace physics {

typedef float real_t;

typedef unsigned int id_t;

using v2 = vvm::v2<real_t>;
using m2 = vvm::m2<real_t>;

struct collision_shape {
    v2 extents;
};

struct transform {
    v2 position;
    real_t angle;
};

struct velocity {
    v2 linear;
    real_t angular;
};

struct aabb {
    v2 min_extent, max_extent;
};


struct collision_world {
    // shapes
    std::vector<collision_shape> collision_shapes;

    // bodies' data
    std::vector<transform> transforms;
    std::vector<id_t> shape_ids;
    std::vector<aabb> aabbs;
    size_t num_bodies;

    // internal data types

    struct intersecting_pair {
        id_t b0, b1;

        id_t contact_ids[2];
        int num_contacts;
    
        v2 axis;
        real_t offset;

        enum class feature_id {
            b0_pos_x, b0_neg_x, b0_pos_y, b0_neg_y,
            b1_pos_x, b1_neg_x, b1_pos_y, b1_neg_y
        } feature;
    };

    struct contact {
        id_t pair_id;
        v2 position;
        real_t depth;
    };

    std::vector<intersecting_pair> pairs;
    std::vector<contact> contacts;

    struct broadphase_data;
    broadphase_data* bd;


    // methods

    // broad phase, only test aabbs, use spatial data
    void find_intersecting_pairs();

    // narrow phase, only test the pairs found in the broad phase
    void find_contacts();

    // ctrs/dtrs
    
    collision_world();
    ~collision_world();

};

};  // namespace physics