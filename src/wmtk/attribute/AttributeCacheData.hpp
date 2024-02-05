#pragma once
#include <Eigen/Core>
#include <cassert>
#include "internal/MapTypes.hpp"

namespace wmtk::attribute {
template <typename T>
class AttributeCacheData
{
public:
    using Vector = typename internal::VectorResult<T>;
    template <typename Derived>
    AttributeCacheData(const Eigen::MatrixBase<Derived>& a)
        : data(a)
    {}
    // for WMTK_ONLY_CACHE_WRITES it's annoying to remove all the bool passed in, easiesr to just
    // let it get elided
    //
    AttributeCacheData() = default;

    AttributeCacheData(AttributeCacheData&&) = default;
    AttributeCacheData(const AttributeCacheData&) = default;
    AttributeCacheData& operator=(AttributeCacheData&&) = default;
    AttributeCacheData& operator=(const AttributeCacheData&) = default;
    typename Vector::MapType data_as_map();
    typename Vector::ConstMapType data_as_const_map() const;

    Vector data;
};
} // namespace wmtk::attribute
#include "AttributeCacheData.hxx"
