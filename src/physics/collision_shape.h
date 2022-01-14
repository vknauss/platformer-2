#pragma once

#include "types.h"


namespace physics {

// namespace collision_shape {

// struct box {
//     v2 extents;
// };

// struct circle {
//     real_t radius;
// };

// enum class type_id {
//     box, circle
// };

// template<typename T>
// constexpr type_id get();




// }

struct box_collision_shape {
    v2 extents;
};

struct circle_collision_shape {
    real_t radius;
};

class collision_shape_type {
public:
    enum class id {
        box, circle
    };
    
    template<typename T>
    static constexpr id get();
};

template<>
constexpr collision_shape_type::id collision_shape_type::get<box_collision_shape>() {
    return collision_shape_type::id::box;
}

template<>
constexpr collision_shape_type::id collision_shape_type::get<circle_collision_shape>() {
    return collision_shape_type::id::circle;
}

}