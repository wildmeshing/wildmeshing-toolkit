#pragma once

#include <Eigen/Core>

#include <map>
#include <vector>
#include "AttributeHandle.hpp"

namespace wmtk {

class Mesh;
template <typename T>
class Accessor;

template <typename T>
class MeshAttributes
{
    friend class Accessor<T>;
    friend class Mesh;

    typedef Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> MapResult;
    typedef Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>> ConstMapResult;

    MeshAttributes();

    AttributeHandle register_attribute(const std::string& name, long size);


    long size() const;
    void reserve(const long size);


protected:
    AttributeHandle attribute_handle(const std::string& name) const;

    ConstMapResult vector_attribute(const AttributeHandle& handle, const long index) const;
    MapResult vector_attribute(const AttributeHandle& handle, const long index);

    T scalar_attribute(const AttributeHandle& handle, const long index) const;
    T& scalar_attribute(const AttributeHandle& handle, const long index);

private:
    std::map<std::string, AttributeHandle> m_handles;
    long initial_stride = -1;

    std::vector<std::vector<T>> m_attributes;
};


} // namespace wmtk
