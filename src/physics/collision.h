#pragma once

#include <cstdint>
#include <vector>

#include "types.h"
#include "collision_shape.h"


namespace physics {

struct collision_object {
    transform tfm;
    id_t shape_id;
};

enum class axis_id : uint8_t {
    x, y
};

enum class face_id : uint8_t {
    pos_x, neg_x, pos_y, neg_y
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

    contact_feature feature;
};

struct contact {
    v2 position;
    real_t depth;
    contact_feature feature;
};

// class box_collision_shape : public collision_shape {
// public:
//     type get_type() const override {
//         return type::box;
//     }

//     v2 extents;
// };

// class circle_collision_shape : public collision_shape {
// public:
//     type get_type() const override {
//         return type::circle;
//     }

//     real_t radius;
// };

class collision_world {
public:

    collision_world();
    ~collision_world();

    void update();

    template<typename collision_shape_t>
    id_t add_collision_shape(const collision_shape_t& shape);

    id_t add_collision_object(const collision_object& obj);

    collision_object get_collision_object(id_t id) const;

    template<typename collision_shape_t>
    collision_world get_collision_shape(id_t id) const;

    const std::vector<collision_pair>& get_pairs() const;

    const std::vector<contact>& get_contacts() const;

private:

    class impl;
    impl* _impl;

};

}