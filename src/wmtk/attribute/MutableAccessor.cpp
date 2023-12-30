#include "MutableAccessor.hpp"


namespace wmtk::attribute {


// template <typename T>
// MutableAccessor<T>::MutableAccessor(
//     Mesh& mesh,
//     const MeshAttributeHandle<T>& handle,
//     AttributeAccessMode mode)
//     : ConstAccessorType(mesh, handle, mode)
//{}

template class MutableAccessor<char>;
template class MutableAccessor<int64_t>;
template class MutableAccessor<double>;
} // namespace wmtk::attribute
