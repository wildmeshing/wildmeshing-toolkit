#include "ConstAccessor.hpp"
#include <wmtk/Mesh.hpp>
#include "AttributeManager.hpp"
#include "AttributeScope.hpp"
#include "AttributeScopeStack.hpp"
#include "MeshAttributes.hpp"

namespace wmtk::attribute {

template <typename T>
ConstAccessor<T>::ConstAccessor(
    const Mesh& mesh,
    const TypedAttributeHandle<T>& handle,
    AttributeAccessMode mode)
    : TupleBaseType(const_cast<Mesh&>(mesh), handle, mode)
{}
//===================================================
// These following methods just forward to to const names
template <typename T>
auto ConstAccessor<T>::vector_attribute(const Tuple& t) const -> ConstMapResult
{
    return const_vector_attribute(t);
}
template <typename T>
T ConstAccessor<T>::scalar_attribute(const Tuple& t) const
{
    return const_scalar_attribute(t);
}
//===================================================


template class ConstAccessor<char>;
template class ConstAccessor<int64_t>;
template class ConstAccessor<double>;
template class ConstAccessor<Rational>;
} // namespace wmtk::attribute
