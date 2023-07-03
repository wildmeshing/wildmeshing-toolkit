#include "AccessorBaseBase.hpp"
#include "Mesh.hpp"
#include "MeshAttributes.hpp"

namespace wmtk {

template <typename T, bool IsConst>
auto AccessorBase<T, IsConst>::attributes() -> MeshAttributesType&
{
    return m_mesh.get_mesh_attributes(m_handle);
}
template <typename T, bool IsConst>
auto AccessorBase<T, IsConst>::attributes() const -> const MeshAttributesType&
{
    return m_mesh.get_mesh_attributes(m_handle);
}

template <typename T, bool IsConst>
auto AccessorBase<T, IsConst>::vector_attribute(const long index) const -> ConstMapResult
{
    auto buffer = attributes().vector_attribute(m_handle.m_base_handle, index);


    return buffer;
}

template <typename T, bool IsConst>
T AccessorBase<T, IsConst>::scalar_attribute(const long index) const
{
    auto value = attributes().scalar_attribute(m_handle.m_base_handle, index);
    return value;
}

template <typename T, bool IsConst>
auto AccessorBase<T, IsConst>::scalar_attribute(const long index) -> TT
{
    auto& value = attributes().scalar_attribute(m_handle.m_base_handle, index);
}
} // namespace wmtk
