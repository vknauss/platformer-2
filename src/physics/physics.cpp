#include "physics.h"

#include <cmath>

#include <vvm/matrix_tfm.hpp>

namespace physics {



// struct separating_axis_data {
//     rigid_body* b0, * b1;
//     v2 normal;
//     float separation;
//     bool contact;
// };

// separating_axis_data compute_separating_axis(rigid_body* b0, rigid_body* b1) {
//     // very sorta borrowed from Erin Catto's Box2D Lite collision code
//     // https://github.com/erincatto/box2d-lite
    
//     separating_axis_data result;
//     result.b0 = b0;
//     result.b1 = b1;
//     result.contact = false;

//     v2 d = b1->state.position - b1->state.position;  // the separation between the centers
    
//     m2 r0 = vvm::rotate(b0->state.angle), r1 = vvm::rotate(b1->state.angle);  // the 2d rotation matrices of the 2 bodies
//     m2 r0T = vvm::transpose(r0), r1T = vvm::transpose(r1);  // the transposes (=inverse) of the previous, used for world-to-body space transform

//     v2 d0 = r0T * d, d1 = r1T * d;  // the separation vector transformed to the rotational frames of the 2 bodies

//     m2 r0to1 = r0T * r1;  // rotation of body 1 relative to body 0
//     m2 ar01 = vvm::abs(r0to1);  // absolute value of above, used to compute separation (see biig comment in the aabb function above)
//     m2 ar10 = vvm::transpose(ar01);  // the transpose of above gives the version for body 0 relative to 1

//     // the separations between nearest faces relative to body 0 and 1, respectively
//     // this is the exact same formula used in box2d lite. knowing how the absolute value works make it sorta make sense lmao
//     // just subtracting the distance from the centers of each body to their edges from the distance vector, but all in body-space
//     v2 s0 = vvm::abs(d0) - b0->shape->extents - ar01 * b1->shape->extents,
//        s1 = vvm::abs(d1) - b1->shape->extents - ar10 * b0->shape->extents;
    
//     // for the above, positive separation means the bodies aren't colliding
//     if (s0.x <= 0 && s0.y <= 0 && s1.x <= 0 && s1.y <= 0) {
//         result.contact = true;
//         // now the separating axis theorem part, find minimum depth among any of the axes
//         // since all the separations are negative, the minimum depth (closest to 0) is actually the max separation
//         // box2d lite has some biasing used to improve coherence. i'm gonna ignore that for now.
        
//         result.normal = d0.x > 0 ? r0[0] : -r0[0];
//         result.separation = s0.x;

//         if (s0.y > result.separation) {
//             result.normal = d0.y > 0 ? r0[1] : -r0[1];
//             result.separation = s0.y;
//         }

//         if (s1.x > result.separation) {
//             result.normal = d1.x > 0 ? r1[0] : -r1[0];
//             result.separation = s1.x;
//         }

//         if (s1.y > result.separation) {
//             result.normal = d1.y > 0 ? r1[1] : -r1[1];
//             result.separation = s1.y;
//         }
//     }
    
//     return result;
// }

// contact_manifold compute_contact(rigid_body* b0, rigid_body* b1) {
//    contact_manifold result;
//    result.b0 = b0;
//    result.b1 = b1;
//    result.num_points = 0;

//    auto sat = compute_separating_axis(b0, b1);
//    if (sat.contact) {

//    }

//    return result;
// }

};  // namespace physics