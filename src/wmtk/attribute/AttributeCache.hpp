#pragma once
#include "internal/AttributeFlatCache.hpp"

namespace wmtk::attribute {

    template <typename T>
        using AttributeCache = internal::AttributeFlatCache<T>;
}
