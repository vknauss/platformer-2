#pragma once

#include <limits>
#include <vvm/vvm.hpp>

namespace physics {

typedef float real_t;

typedef unsigned int id_t;
constexpr id_t ID_NULL = std::numeric_limits<id_t>::max();

using v2 = vvm::v2<real_t>;
using m2 = vvm::m2<real_t>;

struct transform {
    v2 position;
    real_t angle;
};

struct velocity {
    v2 linear;
    real_t angular;
};

}