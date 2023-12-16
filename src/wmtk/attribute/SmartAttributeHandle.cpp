#include "SmartAttributeHandle.hpp"
#include <cassert>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::attribute {
template <typename T>
SmartAttributeHandle<T>::SmartAttributeHandle(Mesh& m, const MeshAttributeHandle<T>& h)
    : m_mesh(&m)
    , m_handle(h)
{}

template <typename T>
SmartAttributeHandle<T>::SmartAttributeHandle(const SmartAttributeHandle<T>& o) = default;
template <typename T>
SmartAttributeHandle<T>::SmartAttributeHandle(SmartAttributeHandle<T>&& o) = default;
template <typename T>
SmartAttributeHandle<T>& SmartAttributeHandle<T>::operator=(const SmartAttributeHandle<T>& o) = default;
template <typename T>
SmartAttributeHandle<T>& SmartAttributeHandle<T>::operator=(SmartAttributeHandle<T>&& o) = default;

template <typename T>
const Mesh& SmartAttributeHandle<T>::mesh() const
{
    assert(m_mesh != nullptr);
    return *m_mesh;
}
template <typename T>
const MeshAttributeHandle<T>& SmartAttributeHandle<T>::handle() const
{
    return m_handle;
}

template <typename T>
SmartAttributeHandle<T>::operator MeshAttributeHandle<T>() const
{
    return m_handle;
}

template class SmartAttributeHandle<char>;
template class SmartAttributeHandle<long>;
template class SmartAttributeHandle<double>;
template class SmartAttributeHandle<Rational>;

} // namespace wmtk::attribute