#pragma once

#include <vvm/vvm.hpp>

#include <cstdint>
#include <limits>
#include <map>
#include <vector>

#include "storage.h"

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

enum class axis_id : uint8_t {
    x, y
};

enum class face_id : uint8_t {
    pos_x, neg_x, pos_y, neg_y
};

enum class feature_id : uint8_t {
    b0_pos_x, b0_neg_x, b0_pos_y, b0_neg_y,
    b1_pos_x, b1_neg_x, b1_pos_y, b1_neg_y
};

struct contact_feature {
    face_id b0_face, b1_face;
    axis_id ref_axis;
    bool b0_reference, point_side_pos;

    bool operator==(const contact_feature& f) const;
};

struct collision_pair {
    id_t i0, i1;

    id_t contact_ids[2];
    int num_contacts;

    v2 normal;
    v2 tangent;

    real_t depth;

    // feature_id reference_face, incident_face;
    contact_feature feature;

    v2 incident_face_points[2];
    v2 reference_face_points[2];
};

// struct pair_cache {
//     std::vector<collision_pair> pairs;
//     std::map<std::pair<id_t, id_t>, id_t> pairs_map;
// };


constexpr id_t ID_NULL = std::numeric_limits<id_t>::max();

// struct rigid_body {
//     real_t mass;
//     id_t shape_id;
//     transform tfm;
//     velocity vel;
// };

// class world {
//     std::vector<collision_shape> shapes;
    
//     std::vector<transform> tfms;
//     std::vector<velocity> vels;


// };

// struct intersecting_pair {
//     id_t b0, b1;

//     id_t contact_ids[2];
//     int num_contacts;

//     v2 axis;
//     real_t offset;

//     feature_id feature;
// };

struct contact {
    // id_t pair_id;
    v2 position;
    real_t depth;
    contact_feature feature;
};


class broadphase {
public:
    using pair = std::pair<id_t, id_t>;

    const std::vector<pair>& pairs() const;

    void update(const std::vector<aabb>& aabbs);

private:
    std::vector<id_t> _sort_ids;
    std::vector<id_t> _intervals;

    std::vector<pair> _pairs;
};

class narrowphase {
public:

    const std::vector<collision_pair>& pairs() const;
    const std::vector<contact>& contacts() const;

    void update(const std::vector<broadphase::pair>& bpairs, 
        const std::vector<collision_shape>& shapes,
        const std::vector<transform>& tfms,
        const std::vector<id_t>& shape_ids);

private:

    std::vector<collision_pair> _pairs;
    std::vector<collision_pair> _old_pairs;

    std::vector<contact> _contacts;

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

struct collision_world {
    // shapes
    std::vector<collision_shape> collision_shapes;

    // bodies' data
    std::vector<transform> transforms;
    std::vector<id_t> shape_ids;
    std::vector<aabb> aabbs;
    size_t num_bodies;

    // std::vector<collision_pair> pairs;
    // std::vector<contact> contacts;

    broadphase bp;
    narrowphase np;

    // methods

    id_t add_collision_shape(const collision_shape& shape);
    id_t add_collision_object(const transform& tfm, id_t shape_id);


    void update();

    // ctrs/dtrs
    
    collision_world();
    virtual ~collision_world();

};

struct contact_constraint {
    id_t i0, i1;
    contact c;
    v2 r0, r1;
    real_t impulse_normal, impulse_tangent;
    real_t mass_normal, mass_tangent;
};

struct contact_solver_pair {
    id_t i0, i1;
    id_t contact_ids[2];
    int num_contacts;
    v2 normal, tangent;
    real_t friction;
};

struct contact_pair_cache {

};

struct contact_solver {
    real_t allowed_penetration = 0.01, bias = 0.2;

    std::vector<contact_constraint> ccs;
    std::vector<contact_solver_pair> sps;
    std::vector<contact_constraint> oldccs;
    std::vector<contact_solver_pair> oldsps;
    std::vector<id_t> sort_ids;

    // mapped_storage<contact_constraint> cmap;

    void update(const std::vector<collision_pair>& ps,
                const std::vector<contact>& cs,
                const std::vector<transform>& tfm,
                const std::vector<real_t>& im,
                const std::vector<real_t>& ii,
                const std::vector<real_t>& friction);

    void solve(const std::vector<real_t>& im, 
               const std::vector<real_t>& ii,
               real_t dt,
               std::vector<velocity>& vels);
};

struct dynamics_world {

    // more body data
    std::vector<velocity> velocities;
    std::vector<real_t> imasses;
    std::vector<real_t> iinertias;
    std::vector<real_t> frictions;

    collision_world cw;

    contact_solver solver;

    size_t num_bodies;
    
    id_t add_collision_shape(const collision_shape& shape);
    id_t add_rigid_body(real_t mass, const transform& tfm, id_t shape_id, real_t friction);

    void step(real_t dt, int iterations);

    dynamics_world();
    virtual ~dynamics_world();

};

};  // namespace physics