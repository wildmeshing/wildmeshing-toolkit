#pragma once

#include "Attribute.hpp"
#include "AttributeHandle.hpp"


#include <Eigen/Core>

#include <map>
#include <vector>


namespace wmtk {

class MeshWriter;
class Mesh;
template <typename T, bool isConst>
class Accessor;

template <typename T>
class MeshAttributes
{
    friend class Accessor<T, false>;
    friend class Accessor<T, true>;
    friend class Mesh;

    typedef Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> MapResult;
    typedef Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>> ConstMapResult;


public:
    MeshAttributes();

    void serialize(const int dim, MeshWriter& writer) const;

    AttributeHandle register_attribute(const std::string& name, long size, bool replace = false);

    long size() const;
    void reserve(const long size);

    bool operator==(const MeshAttributes<T>& other) const;

protected:
    AttributeHandle attribute_handle(const std::string& name) const;

    ConstMapResult vector_attribute(const AttributeHandle& handle, const long index) const;
    MapResult vector_attribute(const AttributeHandle& handle, const long index);

    T scalar_attribute(const AttributeHandle& handle, const long index) const;
    T& scalar_attribute(const AttributeHandle& handle, const long index);

    // pass by value due to
    //https://clang.llvm.org/extra/clang-tidy/checks/modernize/pass-by-value.html
    void set(const AttributeHandle& handle, std::vector<T> val);

private:
    std::map<std::string, AttributeHandle> m_handles;

    long m_size = -1;

    std::vector<Attribute<T>> m_attributes;
};


} // namespace wmtk
