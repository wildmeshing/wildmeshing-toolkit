#pragma once

#include <Eigen/Core>

#include <map>
#include <vector>

namespace wmtk {

class AttributeHandle
{
public:
    long index;
    long stride;
};

template <typename T>
class MeshAttributes
{
    typedef Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> MapResult;
    typedef Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>> ConstMapResult;

    MeshAttributes();

    AttributeHandle register_attribute(const std::string& name, long size);

    AttributeHandle get_attribute_handle(const std::string& name) const;

    const ConstMapResult get_vector_attribute(const std::string& name, const long index) const;
    const ConstMapResult get_vector_attribute(const AttributeHandle& handle, const long index)
        const;

    MapResult get_vector_attribute(const std::string& name, const long index);
    MapResult get_vector_attribute(const AttributeHandle& handle, const long index);

    T get_scalar_attribute(const std::string& name, const long index) const;
    T get_scalar_attribute(const AttributeHandle& handle, const long index) const;

    T& get_scalar_attribute(const std::string& name, const long index);
    T& get_scalar_attribute(const AttributeHandle& handle, const long index);


    long size() const;
    void resize(const long size);

    void rollback();
    void begin_protect();
    void end_protect();
    bool is_in_protect() const;

private:
    std::map<std::string, AttributeHandle> m_handles;
    long initial_stride = -1;

    std::vector<std::vector<T>> m_attributes;
    std::vector<std::vector<T>> m_attributes_copy;

    inline std::vector<std::vector<T>>& current_attributes()
    {
        return m_attributes_copy.empty() ? m_attributes : m_attributes_copy;
    }

    inline const std::vector<std::vector<T>>& current_attributes() const
    {
        return m_attributes_copy.empty() ? m_attributes : m_attributes_copy;
    }
};


} // namespace wmtk