#pragma once
#if defined(WMTK_ENABLE_MAP_CACHE)
#include "internal/AttributeMapCache.hpp"

namespace wmtk::attribute {

    template <typename T>
        using AttributeCache = internal::AttributeMapCache<T>;
}
#else
#include "internal/AttributeFlatCache.hpp"

namespace wmtk::attribute {

    template <typename T>
        using AttributeCache = internal::AttributeFlatCache<T>;
}
#endif
