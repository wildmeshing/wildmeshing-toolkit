#include <cassert>
#include <tuple>
#include <wmtk/utils/Rational.hpp>
#include "MeshAttributeHandle.hpp"

namespace wmtk::attribute {


template <typename T>
bool TypedAttributeHandle<T>::operator<(const TypedAttributeHandle<T>& o) const
{
    return std::tie(m_base_handle, m_primitive_type) <
           std::tie(o.m_base_handle, o.m_primitive_type);
}
template <typename T>
TypedAttributeHandle<T>::TypedAttributeHandle(const MeshAttributeHandle& h)
    : TypedAttributeHandle(h.as<T>())
{
    assert(h.holds<T>());
}

template class TypedAttributeHandle<double>;
template class TypedAttributeHandle<int64_t>;
template class TypedAttributeHandle<Rational>;
template class TypedAttributeHandle<char>;
} // namespace wmtk::attribute
