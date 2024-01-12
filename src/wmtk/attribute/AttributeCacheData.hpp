#pragma once
#include <Eigen/Core>

namespace wmtk {
template <typename T>
class AttributeCacheData
{
public:
    using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1, 0, 6, 1>;
    template <typename Derived>
    AttributeCacheData(const Eigen::MatrixBase<Derived>& a, bool d = false)
        : data(a)
        , dirty(d)
    {}
    AttributeCacheData(bool d = false)
        : dirty(d)
    {}
    typename Vector::MapType data_as_map();
    typename Vector::ConstMapType data_as_const_map() const;

    Vector data;
    bool dirty = false;
};
} // namespace wmtk
