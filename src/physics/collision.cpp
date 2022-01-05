#include "physics.h"

#include <cassert>
#include <cmath>

#include <iostream>

#include <vvm/matrix_tfm.hpp>
#include <vvm/string.hpp>


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

struct sat_data {
    v2 normal;
    real_t depth;
    face_id reference_face;
    axis_id reference_axis;
    bool b0_reference;
};

static sat_data compute_sat(const collision_shape& s0, const collision_shape& s1,
                            const transform& t0, const transform& t1) {
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

    sat_data result;

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
    
    result.b0_reference = true;

    // test b0 x axis
    auto depth = result.depth = std::abs(d0.x) - s0.extents.x - e10.x;
    result.normal = r0[0];
    result.reference_face = face_id::pos_x;
    result.reference_axis = axis_id::x;
    if (d0.x < 0) {
        result.normal = -result.normal;
        result.reference_face = face_id::neg_x;
    }
    if (depth > 0) {  // separating axis found
        return result;
    }

    // test b0 y axis
    depth = std::abs(d0.y) - s0.extents.y - e10.y;
    if (depth > result.depth) {
        result.depth = depth;
        result.normal = r0[1];
        result.reference_face = face_id::pos_y;
        result.reference_axis = axis_id::y;
        if (d0.y < 0) {
            result.normal = -result.normal;
            result.reference_face = face_id::neg_y;
        }
        if (depth > 0) return result;
    }

    // test b1 axes
    // we can just transpose ar10 to get abs(r1t * r0)
    m2 ar01 = vvm::transpose(ar10);
    v2 e01 = ar01 * s0.extents;


    // test b1 x axis
    depth = std::abs(d1.x) - s1.extents.x - e01.x;
    if (depth > result.depth) {
        result.depth = depth;
        result.b0_reference = false;
        result.reference_axis = axis_id::x;
        result.normal = -r1[0];
        result.reference_face = face_id::pos_x;  // inverted since d points from 0 to 1
        if (d1.x > 0) {
            result.normal = -result.normal;
            result.reference_face = face_id::neg_x;
        }
        if (depth > 0) return result;
    }

    // test b1 y axis
    depth = std::abs(d1.y) - s1.extents.y - e01.y;
    if (depth > result.depth) {
        result.depth = depth;
        result.b0_reference = false;
        result.reference_axis = axis_id::y;
        result.normal = -r1[1];
        result.reference_face = face_id::pos_y;  // inverted since d points from 0 to 1
        if (d1.y > 0) {
            result.normal = -result.normal;
            result.reference_face = face_id::neg_y;
        }
    }

    return result;
}

static face_id find_incident(const v2& normal, const m2& rot_inc) {
    real_t d0 = vvm::dot(rot_inc[0], normal), d1 = vvm::dot(rot_inc[1], normal);
    if (std::abs(d0) > std::abs(d1)) {
        return d0 > 0 ? face_id::neg_x : face_id::pos_x;
    }
    return d1 > 0 ? face_id::neg_y : face_id::pos_y;
}

struct a_couple_points {
    v2 points[2];
    int num_points = 0;
};

static constexpr a_couple_points edge_points(face_id f, const v2& extents, const m2& rot, const v2& pos) {
    switch (f) {
    case face_id::neg_x:
        return {
            { pos + rot * v2(-extents.x, -extents.y), 
              pos + rot * v2(-extents.x,  extents.y)
            }, 2};
    case face_id::neg_y:
        return {
            { pos + rot * v2(-extents.x, -extents.y), 
              pos + rot * v2( extents.x, -extents.y)
            }, 2};
    case face_id::pos_x:
        return {
            { pos + rot * v2( extents.x, -extents.y), 
              pos + rot * v2( extents.x,  extents.y)
            }, 2};
    case face_id::pos_y:
        return {
            { pos + rot * v2(-extents.x,  extents.y), 
              pos + rot * v2( extents.x,  extents.y)
            }, 2};
    }
    return {{}, 0};
}

a_couple_points clip_points(const a_couple_points& in_points, const v2& tangent_axis, real_t min, real_t max) {
    a_couple_points out_points;
    int out_side[2] {0, 0};
    real_t d[2] {0, 0};
    for (int i = 0; i < in_points.num_points; ++i) {
        d[i] = vvm::dot(tangent_axis, in_points.points[i]);
        out_side[i] = d[i] > max ? 1 : (d[i] < min ? -1 : 0);
    }
    if (in_points.num_points > 1 && out_side[0] * out_side[1] != 1) {
        for (int i = 0; i < in_points.num_points; ++i) {
            out_points.points[i] = in_points.points[i];
            if (out_side[i] == -1) {
                real_t interp = (min - d[i]) / (d[1-i] - d[i]);
                out_points.points[i] = vvm::lerp(in_points.points[i], in_points.points[1-i], interp);
            }
            if (out_side[i] == 1) {
                real_t interp = (max - d[1-i]) / (d[i] - d[1-i]);
                out_points.points[i] = vvm::lerp(in_points.points[1-i], in_points.points[i], interp);
            }
        }
        out_points.num_points = in_points.num_points;
    }
    if (in_points.num_points == 1 && out_side[0] == 0) {
        out_points.points[0] = in_points.points[0];
        out_points.num_points = 1;
    }
    // if (out_points.num_points == 0) {
       // just for testing
    // }
    return out_points;
}

struct clip_data {
    v2 axis;
    real_t min, max;
};

// these are the most disgusting function signatures ever
// definitely need a refactor at some point
// these should maybe be class methods, possibly using pimpl
// return the number of contact points generated
static int find_contact_points(contact c[2], const collision_pair& p, const collision_shape& s0, const collision_shape& s1, const transform& t0, const transform& t1) {
    const v2& p0 = t0.position;
    const v2& p1 = t1.position;
    
    a_couple_points incident_points;
    if (p.feature.b0_reference) { 
        incident_points = edge_points(p.feature.b1_face, s1.extents, vvm::rotate(t1.angle), p1);
    } else {
        incident_points = edge_points(p.feature.b0_face, s0.extents, vvm::rotate(t0.angle), p0);
    }

    real_t min, max;
    real_t front;
    v2 axis;

    if (p.feature.b0_reference) {
        axis = p.normal;
        real_t pd0 = vvm::dot(p.tangent, p0);
        switch (p.feature.ref_axis) {
        case axis_id::x:
            min = pd0 - s0.extents.y;
            max = pd0 + s0.extents.y;
            front = dot(axis, p0) + s0.extents.x;
            break;
        case axis_id::y:
            min = pd0 - s0.extents.x;
            max = pd0 + s0.extents.x;
            front = dot(axis, p0) + s0.extents.y;
            break;
        }
    } else {
        axis = -p.normal;
        real_t pd1 = vvm::dot(p.tangent, p1);
        switch (p.feature.ref_axis) {
        case axis_id::x:
            min = pd1 - s1.extents.y;
            max = pd1 + s1.extents.y;
            front = dot(axis, p1) + s1.extents.x;
            break;
        case axis_id::y:
            min = pd1 - s1.extents.x;
            max = pd1 + s1.extents.x;
            front = dot(axis, p1) + s1.extents.y;
            break;
        }
    }

    auto clipped = clip_points(incident_points, p.tangent, min, max);
    
    int num_out = 0;
    for (int i = 0; i < clipped.num_points; ++i) {
        real_t depth = vvm::dot(axis, clipped.points[i]) - front;
        if (depth <= 0) {
            auto& ct = c[num_out++];
            ct.position = clipped.points[i];
            ct.depth = depth;
            ct.feature = p.feature;
            ct.feature.point_side_pos = (i == 1);  // is this the positive side clip, clipping maintains point order and edge_points return neg first then pos
        }
    }
    return num_out;
}

void broadphase::update(const std::vector<aabb>& aabbs) {
    
    // 2 axis sort-and-sweep
    static const auto compare_x = [] (const aabb& b0, const aabb& b1) {
        return b0.min_extent.x < b1.min_extent.x;
    };

    static const auto compare_y = [] (const aabb& b0, const aabb& b1) {
        return b0.min_extent.y < b1.min_extent.y;
    };

    // for now, only worry about sorting along 1 axis

    if (_sort_ids.size() != aabbs.size()) {
        _sort_ids.resize(aabbs.size());
        for (auto i = 0u; i < aabbs.size(); ++i) {
            _sort_ids[i] = i;
        }
    }
    _sort_ids.resize(aabbs.size());

    // sort a vector of ids by using a compare function on elements of another vector indexed by the ids
    sort_mapped_ids(_sort_ids, aabbs, compare_x);

    _pairs.clear();
    _intervals.clear();

    // iterate over the aabbs in sorted order, which is lowest to highest min x coord
    // keep track of the current min x coord, and maintain a list of indices of bodies whose max x
    // coord is at least as large as the current min x coord
    // each time a body is checked in sorted order, add a pair for each body in the active intervals list
    for (auto id0 : _sort_ids) {
        // remove inactive intervals
        auto ni = 0u;
        const auto& b0 = aabbs[id0];
        for (auto i = 0u; i < _intervals.size(); ++i) {
            auto id1 = _intervals[i];
            const auto& b1 = aabbs[id1];
            if (b1.max_extent.x < b0.min_extent.x) {
                continue;
            }
            _intervals[ni++] = id1;
        }
        _intervals.resize(ni);

        // test aabbs and add pairs
        for (auto id1 : _intervals) {
            const auto& b1 = aabbs[id1];
            if (aabbs_intersect(b0, b1)) {
                _pairs.push_back(id0 < id1 ? pair {id0, id1} : pair {id1, id0});
            }
        }

        _intervals.push_back(id0);
    }
}



void narrowphase::update(const std::vector<broadphase::pair>& bpairs, const std::vector<collision_shape>& shapes,
        const std::vector<transform>& tfms, const std::vector<id_t>& shape_ids) {
    _old_pairs = _pairs;
    _pairs.clear();
    _contacts.clear();

    for (auto i = 0u; i < bpairs.size(); ++i) {
        const auto& [i0, i1] = bpairs[i];
        const auto& t0 = tfms[i0], & t1 = tfms[i1];
        const auto& s0 = shapes[shape_ids[i0]], & s1 = shapes[shape_ids[i1]];

        auto sat_result = compute_sat(s0, s1, t0, t1);
        if (sat_result.depth <= 0.0) {
            collision_pair p;
            p.i0 = i0;
            p.i1 = i1;
            p.normal = sat_result.normal;
            p.depth = sat_result.depth;
            if (sat_result.b0_reference) {
                p.feature.b0_face = sat_result.reference_face;
                p.feature.b1_face = find_incident(sat_result.normal, vvm::rotate(t1.angle));
            } else {
                p.feature.b1_face = sat_result.reference_face;
                p.feature.b0_face = find_incident(-sat_result.normal, vvm::rotate(t0.angle));
            }
            p.feature.b0_reference = sat_result.b0_reference;
            p.feature.ref_axis = sat_result.reference_axis;
            p.tangent = {p.normal.y, -p.normal.x};
            

            contact c[2];
            p.num_contacts = find_contact_points(c, p, s0, s1, t0, t1);

            auto b0_points = edge_points(p.feature.b0_face, s0.extents, vvm::rotate(t0.angle), t0.position);
            auto b1_points = edge_points(p.feature.b1_face, s1.extents, vvm::rotate(t1.angle), t1.position);
            if (p.feature.b0_reference) {
                p.reference_face_points[0] = b0_points.points[0];
                p.reference_face_points[1] = b0_points.points[1];
                p.incident_face_points[0] = b1_points.points[0];
                p.incident_face_points[1] = b1_points.points[1];
            } else {
                p.reference_face_points[0] = b1_points.points[0];
                p.reference_face_points[1] = b1_points.points[1];
                p.incident_face_points[0] = b0_points.points[0];
                p.incident_face_points[1] = b0_points.points[1];
            }

            for (int j = 0; j < p.num_contacts; ++j) {
                p.contact_ids[j] = (id_t) _contacts.size();
                _contacts.push_back(c[j]);
            }

            _pairs.push_back(p);
        }
    }
}

void collision_world::update() {
    // first update all aabbs
    aabbs.resize(num_bodies);
    for (auto i = 0u; i < num_bodies; ++i) {
        aabbs[i] = get_shape_aabb(collision_shapes[shape_ids[i]], transforms[i]);
    }
    
    // run broadphase
    bp.update(aabbs);

    // run narrow phase
    np.update(bp.pairs(), collision_shapes, transforms, shape_ids);
}

id_t collision_world::add_collision_shape(const collision_shape& shape) {
    id_t id = collision_shapes.size();
    collision_shapes.push_back(shape);
    return id;
}

id_t collision_world::add_collision_object(const transform& tfm, id_t shape_id) {
    assert(shape_id < collision_shapes.size());
    id_t id = num_bodies++;
    transforms.push_back(tfm);
    shape_ids.push_back(shape_id);
    return id;
}

collision_world::collision_world() :
    num_bodies(0) {
}

collision_world::~collision_world() {
}

};  // namespace physics