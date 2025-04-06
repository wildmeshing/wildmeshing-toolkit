#include <fmt/format.h>
#include <cassert>
#include <tuple>
#include <wmtk/utils/Rational.hpp>
#include "MeshAttributeHandle.hpp"

namespace wmtk::attribute {

template <typename T>
TypedAttributeHandle<T>::operator std::string() const
{
    return fmt::format(
        "{}:{}",
        m_base_handle.index(),
        wmtk::primitive_type_name(m_primitive_type)

    );
}

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
