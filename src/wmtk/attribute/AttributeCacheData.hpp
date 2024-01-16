#pragma once
#include <Eigen/Core>

namespace wmtk {
template <typename T>
class AttributeCacheData
{
public:
    using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1, 0, WMTK_MAX_ATTRIBUTE_DIMENSION, 1>;
    template <typename Derived>
    AttributeCacheData(const Eigen::MatrixBase<Derived>& a, bool d = false)
        : data(a)
#if !defined(WMTK_ONLY_CACHE_WRITES)
        , dirty(d)
#endif
    {}
    // for WMTK_ONLY_CACHE_WRITES it's annoying to remove all the bool passed in, easiesr to just let it get elided
    AttributeCacheData(bool d = false)
#if !defined(WMTK_ONLY_CACHE_WRITES)
        : dirty(d)
#endif
    {}
    typename Vector::MapType data_as_map();
    typename Vector::ConstMapType data_as_const_map() const;

    Vector data;
#if !defined(WMTK_ONLY_CACHE_WRITES)
    bool dirty = false;
    #endif
};
} // namespace wmtk
