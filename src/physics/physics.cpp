#include "physics.h"

#include <cmath>

#include <vvm/matrix_tfm.hpp>

namespace physics {

constexpr aabb get_shape_aabb(const collision_shape &shape, const v2 &position, real_t angle) {
    // showing my work:
    // so (for a box) we have 4 points, and they're symmetric (like a box)
    // we only really care about the max size of the transformed points
    // we can get the min easily using symmetry
    // let's look at the rotation matrix:
    // rot(a) = [ [ cos(a), sin(a) ], [ -sin(a), cos(a) ] ] (column major)
    // now our points: { [-x, -y], [x, -y], [-x, y], [x, y] }
    // rotated: (im bad at matrix multiplication lmao, let's go slow, 1 by 1)
    //  [-x * cos(a) + y * sin(a), -x * sin(a) - y * cos(a) ]
    //  [ x * cos(a) + y * sin(a),  x * sin(a) - y * cos(a) ]
    //  [-x * cos(a) - y * sin(a), -x * sin(a) + y * cos(a) ]
    //  [ x * cos(a) - y * sin(a),  x * sin(a) + y * cos(a) ]  think i got that right
    // so what's gonna be the max of this? we don't immediately know the sign of cos(a) and sin(a)
    // x and y we can assume are positive
    // case 1: cos(a) >= 0, sin(a) >= 0:
    //  max = [xcos(a) + ysin(a), xsin(a) + ycos(a)]
    // case 2: cos(a) < 0, sin(a) >= 0:
    //  max = [-xcos(a) + ysin(a), xsin(a) - ycos(a)]
    // case 3: cos(a) >= 0, sin(a) < 0:
    //  max = [xcos(a) - ysin(a), -xsin(a) + ycos(a)]
    // case 4: cos(a) < 0, sin(a) < 0:
    //  max = [-xcos(a) - ysin(a), -xsin(a) + ycos(a)]
    // now given that abs(x) = x if x >= 0 else -x, in all cases:
    // max = [x * abs(cos(a)) + y * abs(sin(a)), x * abs(sin(a)) + y * abs(cos(a))]
    // so if we define a simple element-wise abs of a matrix,
    // the max transformed coords is simply abs(rot(a)) * [x, y]
    // NOW Erin Catto's code makes sense lmao.... the magic of comments
    // ok so now we can implement:

    m2 rot = vvm::rotate(angle);
    m2 absRot = vvm::abs(rot);

    aabb b;
    b.max_extent = absRot * shape.extents;
    b.min_extent = position - b.max_extent;
    b.max_extent += position;

    return b;
}

constexpr bool aabbs_intersect(const aabb &b0, const aabb &b1) {
    return b0.max_extent.x >= b1.min_extent.x && b0.min_extent.x <= b1.max_extent.x &&
        b0.max_extent.y >= b1.min_extent.y && b0.min_extent.y <= b1.max_extent.y;
}

struct separating_axis_data {
    rigid_body* b0, * b1;
    v2 normal;
    float separation;
    bool contact;
};

separating_axis_data compute_separating_axis(rigid_body* b0, rigid_body* b1) {
    // very sorta borrowed from Erin Catto's Box2D Lite collision code
    // https://github.com/erincatto/box2d-lite
    
    separating_axis_data result;
    result.b0 = b0;
    result.b1 = b1;
    result.contact = false;

    v2 d = b1->state.position - b1->state.position;  // the separation between the centers
    
    m2 r0 = vvm::rotate(b0->state.angle), r1 = vvm::rotate(b1->state.angle);  // the 2d rotation matrices of the 2 bodies
    m2 r0T = vvm::transpose(r0), r1T = vvm::transpose(r1);  // the transposes (=inverse) of the previous, used for world-to-body space transform

    v2 d0 = r0T * d, d1 = r1T * d;  // the separation vector transformed to the rotational frames of the 2 bodies

    m2 r0to1 = r0T * r1;  // rotation of body 1 relative to body 0
    m2 ar01 = vvm::abs(r0to1);  // absolute value of above, used to compute separation (see biig comment in the aabb function above)
    m2 ar10 = vvm::transpose(ar01);  // the transpose of above gives the version for body 0 relative to 1

    // the separations between nearest faces relative to body 0 and 1, respectively
    // this is the exact same formula used in box2d lite. knowing how the absolute value works make it sorta make sense lmao
    // just subtracting the distance from the centers of each body to their edges from the distance vector, but all in body-space
    v2 s0 = vvm::abs(d0) - b0->shape->extents - ar01 * b1->shape->extents,
       s1 = vvm::abs(d1) - b1->shape->extents - ar10 * b0->shape->extents;
    
    // for the above, positive separation means the bodies aren't colliding
    if (s0.x <= 0 && s0.y <= 0 && s1.x <= 0 && s1.y <= 0) {
        result.contact = true;
        // now the separating axis theorem part, find minimum depth among any of the axes
        // since all the separations are negative, the minimum depth (closest to 0) is actually the max separation
        // box2d lite has some biasing used to improve coherence. i'm gonna ignore that for now.
        
        result.normal = d0.x > 0 ? r0[0] : -r0[0];
        result.separation = s0.x;

        if (s0.y > result.separation) {
            result.normal = d0.y > 0 ? r0[1] : -r0[1];
            result.separation = s0.y;
        }

        if (s1.x > result.separation) {
            result.normal = d1.x > 0 ? r1[0] : -r1[0];
            result.separation = s1.x;
        }

        if (s1.y > result.separation) {
            result.normal = d1.y > 0 ? r1[1] : -r1[1];
            result.separation = s1.y;
        }
    }
    
    return result;
}

contact_manifold compute_contact(rigid_body* b0, rigid_body* b1) {
   contact_manifold result;
   result.b0 = b0;
   result.b1 = b1;
   result.num_points = 0;

   auto sat = compute_separating_axis(b0, b1);
   if (sat.contact) {

   }

   return result;
}

};  // namespace physics