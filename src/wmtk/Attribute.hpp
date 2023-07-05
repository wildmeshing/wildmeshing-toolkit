#pragma once

#include <Eigen/Core>
#include <vector>

namespace wmtk {
class MeshWriter;
template <typename T>
class Attribute
{
public:
    using MapResult = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>>;
    using ConstMapResult = Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>>;

    void serialize(const std::string& name, const int dim, MeshWriter& writer) const;

    // if size < 0 then the internal data is not initialized
    Attribute(long stride, long size);
    ConstMapResult const_vector_attribute(const long index) const;
    MapResult vector_attribute(const long index);

    T const_scalar_attribute(const long index) const;
    T& scalar_attribute(const long index);

    void set(std::vector<T> val);
    long size() const;
    long stride() const;
    void reserve(const long size);

    bool operator==(const Attribute<T>& o) const;

private:
    std::vector<T> m_data;
    long m_stride = -1;
};
} // namespace wmtk
