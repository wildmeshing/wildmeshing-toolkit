#include "MeshAttributeHandle.hpp"
#include <cassert>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::attribute {
template <typename T>
MeshAttributeHandle<T>::MeshAttributeHandle(Mesh& m, const TypedAttributeHandle<T>& h)
    : TypedAttributeHandle<T>(h)
    , m_mesh(&m)
{}
template <typename T>
MeshAttributeHandle<T>::MeshAttributeHandle() = default;

template <typename T>
MeshAttributeHandle<T>::MeshAttributeHandle(const MeshAttributeHandle<T>& o) = default;
template <typename T>
MeshAttributeHandle<T>::MeshAttributeHandle(MeshAttributeHandle<T>&& o) = default;
template <typename T>
MeshAttributeHandle<T>& MeshAttributeHandle<T>::operator=(const MeshAttributeHandle<T>& o) =
    default;
template <typename T>
MeshAttributeHandle<T>& MeshAttributeHandle<T>::operator=(MeshAttributeHandle<T>&& o) = default;

template <typename T>
const Mesh& MeshAttributeHandle<T>::mesh() const
{
    assert(m_mesh != nullptr);
    return *m_mesh;
}
template <typename T>
Mesh& MeshAttributeHandle<T>::mesh()
{
    assert(m_mesh != nullptr);
    return *m_mesh;
}

template <typename T>
int64_t MeshAttributeHandle<T>::dimension() const
{
    return mesh().get_attribute_dimension(*this);
}

template class MeshAttributeHandle<char>;
template class MeshAttributeHandle<int64_t>;
template class MeshAttributeHandle<double>;
template class MeshAttributeHandle<Rational>;

} // namespace wmtk::attribute
