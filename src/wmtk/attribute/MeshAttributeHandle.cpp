#include "MeshAttributeHandle.hpp"
#include <cassert>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::attribute {
template <typename T>
MeshAttributeHandle::MeshAttributeHandle(Mesh& m, const TypedAttributeHandleVariant& h)
    : : m_mesh(&m), m_handle(h)
{}
MeshAttributeHandle::MeshAttributeHandle() = default;

MeshAttributeHandle::MeshAttributeHandle(const MeshAttributeHandle<T>& o) = default;
MeshAttributeHandle::MeshAttributeHandle(MeshAttributeHandle<T>&& o) = default;
MeshAttributeHandle& MeshAttributeHandle::operator=(const MeshAttributeHandle& o) = default;
MeshAttributeHandle& MeshAttributeHandle::operator=(MeshAttributeHandle<T>&& o) = default;

bool MeshAttributeHandle::is_same_mesh(const Mesh& m) const
{
    assert(m_mesh != nullptr);
    return m_mesh == &m;
}


} // namespace wmtk::attribute
