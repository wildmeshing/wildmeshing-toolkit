#pragma once
#include <Eigen/Core>
#include <cassert>

namespace wmtk {
template <typename T>
class AttributeCacheData
{
public:
    using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1, 0, WMTK_MAX_ATTRIBUTE_DIMENSION, 1>;
#if !defined(WMTK_ONLY_CACHE_WRITES)
    template <typename Derived>
    AttributeCacheData(const Eigen::MatrixBase<Derived>& a, bool d = false)
        : data(a)
        , dirty(d)
    {}
    // for WMTK_ONLY_CACHE_WRITES it's annoying to remove all the bool passed in, easiesr to just
    // let it get elided
    AttributeCacheData(bool d = false)
        : dirty(d)
    {}
#else
    template <typename Derived>
    AttributeCacheData(const Eigen::MatrixBase<Derived>& a)
        : data(a)
    {}
    // for WMTK_ONLY_CACHE_WRITES it's annoying to remove all the bool passed in, easiesr to just
    // let it get elided
    //
    AttributeCacheData() = default;
#endif

    AttributeCacheData(AttributeCacheData&&) = default;
    AttributeCacheData(const AttributeCacheData&) = default;
    AttributeCacheData& operator=(AttributeCacheData&&) = default;
    AttributeCacheData& operator=(const AttributeCacheData&) = default;
    typename Vector::MapType data_as_map();
    typename Vector::ConstMapType data_as_const_map() const;

    Vector data;
#if !defined(WMTK_ONLY_CACHE_WRITES)
    bool dirty = false;
#endif
};
} // namespace wmtk
#include "AttributeCacheData.hxx"
