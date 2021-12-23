#include "physics.h"

#include <cassert>
#include <cmath>

#include <iostream>

#include <vvm/matrix_tfm.hpp>


namespace physics {


// showing my work:
// if every shape is a box transformed around its center,
// we only really care about the max size of the transformed points
// we can get the min easily using symmetry
// let's look at the rotation matrix:
// rot(a) = [ [ cos(a), sin(a) ], [ -sin(a), cos(a) ] ] (column major)
// now our points: { [-x, -y], [x, -y], [-x, y], [x, y] }, [x, y] >=0 are the extents
// let sa = sin(a), ca = cos(a)
// rotated points:
//  { [-x*ca + y*sa, -x*sa - y*ca ], [ x*ca + y*sa,  x*sa - y*ca ]
//    [-x*ca - y*sa, -x*sa + y*ca ], [ x*ca - y*sa,  x*sa + y*ca ] }
// the max x and y coords are going to be where the sign of the sin and cos make both terms of the coord positive
// that is, max = [abs(x*ca) + abs(y*sa), abs(x*sa) + abs(y*ca)]
// since x and y are positive, max = [x*abs(ca) + y*abs(sa), x*abs(sa) + y*abs(ca)]
// therefore: max = abs(rot(a)) * [x, y]
// the aabb is then: { position - max, position + max }
static constexpr aabb get_shape_aabb(const collision_shape& shape, const transform& t) {
    m2 rot = vvm::rotate(t.angle);
    m2 absRot = vvm::abs(rot);

    aabb b;
    b.max_extent = absRot * shape.extents;
    b.min_extent = t.position - b.max_extent;
    b.max_extent += t.position;

    return b;
}

static constexpr bool aabbs_intersect(const aabb& b0, const aabb& b1) {
    return b0.max_extent.x >= b1.min_extent.x && b0.min_extent.x <= b1.max_extent.x &&
        b0.max_extent.y >= b1.min_extent.y && b0.min_extent.y <= b1.max_extent.y;
}

struct collision_world::broadphase_data {
    std::vector<id_t> sort_ids_x, sort_ids_y;
    std::vector<id_t> active_intervals;
};

// custom insertion sort
template<typename T, typename compare>
static void sort_mapped_ids(std::vector<id_t>& ids, const std::vector<T>& mapped, const compare& c) {
    assert(ids.size() == mapped.size() && "sort_mapped_ids: array sizes differ.");

    for (auto i = 0u; i < ids.size(); ++i) {
        id_t id0 = ids[i];
        // iterate backwards, swap ids if mapped values are out of order, break otherwise
        for (auto j = 0u; j < i; ++j) {
            id_t id1 = ids[i-j-1];
            if (c(mapped[id0], mapped[id1])) {
                ids[i-j-1] = id0;
                ids[i-j] = id1;
                continue;
            }
            break;
        }
    }
}

void collision_world::find_intersecting_pairs() {
    // first update all aabbs, consider separating this part?
    aabbs.resize(num_bodies);
    for (auto i = 0u; i < num_bodies; ++i) {
        aabbs[i] = get_shape_aabb(collision_shapes[shape_ids[i]], transforms[i]);
    }
    
    // 2 axis sort-and-sweep
    static const auto compare_x = [] (const aabb& b0, const aabb& b1) {
        return b0.min_extent.x < b1.min_extent.x;
    };

    static const auto compare_y = [] (const aabb& b0, const aabb& b1) {
        return b0.min_extent.y < b1.min_extent.y;
    };

    // for now, only worry about sorting along 1 axis
    auto& sort_ids = bd->sort_ids_x;
    auto& intervals = bd->active_intervals;

    if (sort_ids.size() < num_bodies) {
        auto sz = sort_ids.size();
        sort_ids.resize(num_bodies);
        for (auto i = sz; i < num_bodies; ++i) {
            sort_ids[i] = i;
        }
    }
    sort_ids.resize(num_bodies);

    sort_mapped_ids(sort_ids, aabbs, compare_x);

    pairs.clear();
    intervals.clear();

    for (auto id0 : sort_ids) {
        // remove inactive intervals
        auto ni = 0u;
        const auto& b0 = aabbs[id0];
        for (auto i = 0u; i < intervals.size(); ++i) {
            auto id1 = intervals[i];
            const auto& b1 = aabbs[id1];
            if (b1.max_extent.x < b0.min_extent.x) {
                continue;
            }
            intervals[ni++] = id1;
        }
        intervals.resize(ni);

        // test aabbs and add pairs
        for (auto id1 : intervals) {
            const auto& b1 = aabbs[id1];
            if (aabbs_intersect(b0, b1)) {
                intersecting_pair p;
                p.b0 = id0;
                p.b1 = id1;
                if (id1 < id0) {  // use a strict ordering for some reason
                    p.b0 = id1;
                    p.b1 = id0;
                }
                // no persistence of contacts yet... deal with this later
                p.num_contacts = 0;

                pairs.push_back(p);
            }
        }

        intervals.push_back(id0);
    }
}

static bool find_axis(collision_world::intersecting_pair& pair, const collision_world& world) {
    using fid = collision_world::intersecting_pair::feature_id;
    // separating axis theorem
    // test the projection of each shape onto each of the face normal axes of both shapes
    // if we find an axis where the projections do not overlap, store that axis and return no collision
    // otherwise, store the axis of minimum intersection and return collision

    // since our shapes are rectangles, each only has 2 different axes to test, the local x and y
    // the projection of a shape onto its own local x is its width, and y is height

    // the local x axis of a body is the first column of its rotation matrix, y is second column

    // calculating the length of the projection of a body onto a local axis of another body is very similar to
    // how the aabb transformation function above works.
    
    // call the body whose axis is being tested b0, and the other body b1.
    // first, imagine we rotate the space into the local frame of b0, such that b0's x (for instance) is the same as global x
    // then the projection of b1 onto b0's x is just the x components of the aabb of b1

    const transform& t0 = world.transforms[pair.b0];
    const transform& t1 = world.transforms[pair.b1];

    const collision_shape& s0 = world.collision_shapes[world.shape_ids[pair.b0]];
    const collision_shape& s1 = world.collision_shapes[world.shape_ids[pair.b1]];

    m2 r0 = vvm::rotate(t0.angle);  // rotation matrices,
    m2 r1 = vvm::rotate(t1.angle);  // transform points from body space to world space (minus a translation)

    m2 r0t = vvm::transpose(r0);  // transpose (equivalently, inverse) rotation matrices
    m2 r1t = vvm::transpose(r1);  // do the opposite of above (cancel them out)

    v2 d = t1.position - t0.position;  // distance vector from b0 to b1
    v2 d0 = r0t * d;  // d relative to b0
    v2 d1 = r1t * d;  // d relative to b1

    // first test b0 axes
    // the rotation matrix from b1 space to b0 space is r0t * r1
    // by absing this matrix, we can get the length of the projections of b1 onto the axes of b0
    m2 r1to0 = r0t * r1;
    m2 ar10 = vvm::abs(r1to0);
    v2 e10 = ar10 * s1.extents;  // the half extents of b1 in the rotated frame of b0
    
    // test b0 x axis
    float depth = std::abs(d0.x) - s0.extents.x - e10.x;
    float min_depth = depth;
    pair.axis = r0[0];
    pair.feature = fid::b0_pos_x;
    if (d0.x < 0) {
        pair.axis = -pair.axis;
        pair.feature = fid::b0_neg_x;
    }
    pair.offset = vvm::dot(pair.axis, t0.position) + s0.extents.x;
    if (depth > 0) {  // separating axis found
        return false;
    }

    // test b0 y axis
    depth = std::abs(d0.y) - s0.extents.y - e10.y;
    if (depth > min_depth) {
        min_depth = depth;
        pair.axis = r0[1];
        pair.feature = fid::b0_pos_y;
        if (d0.y < 0) {
            pair.axis = -pair.axis;
            pair.feature = fid::b0_neg_y;
        }
        pair.offset = vvm::dot(pair.axis, t0.position) + s0.extents.y;
        if (depth > 0) return false;
    }

    // test b1 axes
    // we can just transpose ar10 to get abs(r1t * r0)
    m2 ar01 = vvm::transpose(ar10);
    v2 e01 = ar01 * s0.extents;

    // test b1 x axis
    depth = std::abs(d1.x) - s1.extents.x - e01.x;
    if (depth > min_depth) {
        min_depth = depth;
        pair.axis = -r1[0];
        pair.feature = fid::b1_neg_x;  // inverted since d points from 0 to 1
        if (d1.x < 0) {
            pair.axis = -pair.axis;
            pair.feature = fid::b1_pos_x;
        }
        pair.offset = vvm::dot(pair.axis, t1.position) - s1.extents.x;
        if (depth > 0) return false;
    }

    // test b1 y axis
    depth = std::abs(d1.y) - s1.extents.y - e01.y;
    if (depth > min_depth) {
        min_depth = depth;
        pair.axis = -r1[1];
        pair.feature = fid::b1_neg_y;  // inverted since d points from 0 to 1
        if (d1.y < 0) {
            pair.axis = -pair.axis;
            pair.feature = fid::b1_pos_y;
        }
        pair.offset = vvm::dot(pair.axis, t1.position) - s1.extents.y;
        if (depth > 0) return false;
    }

    return true;
}

void collision_world::find_contacts() {
    // use the intersecting pairs to find contacts
    for (auto& pair : pairs) {
        if (find_axis(pair, *this)) {
            // do something
            pair.num_contacts++;
        }
    }
}


collision_world::collision_world() :
    num_bodies(0) {
    bd = new broadphase_data;
}

collision_world::~collision_world() {
    delete bd;
}

};  // namespace physics