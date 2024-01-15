#pragma once
#include <Eigen/Core>
#include <cassert>

namespace wmtk {
template <typename T>
class AttributeCacheData
{
public:
    using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1, 0, WMTK_MAX_ATTRIBUTE_DIMENSION, 1>;
    template <typename Derived>
    AttributeCacheData(const Eigen::MatrixBase<Derived>& a, bool d = false)
        : data(a)
        , dirty(d)
    {}
    AttributeCacheData(bool d = false)
        : dirty(d)
    {}

    AttributeCacheData& operator=(const AttributeCacheData& o)
    {
        assert(data.size() == o.data.size());
        data.noalias() = o.data;
        dirty = o.dirty;
        return *this;
    }
    typename Vector::MapType data_as_map();
    typename Vector::ConstMapType data_as_const_map() const;

    Vector data;
    bool dirty = false;
};
} // namespace wmtk
