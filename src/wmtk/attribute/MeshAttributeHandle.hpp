#pragma once
#include "TypedAttributeHandle.hpp"

namespace wmtk {
class Mesh;
}

namespace wmtk::attribute {
template <typename T>
class MutableAccessor;
template <typename T>
class ConstAccessor;

/* @brief Handle that can construct an accessor on its own
 * NOTE: This naming is inconsistent with the existing
 * AttributeHandle/MeshAttributeHandle nomenclature, but in the future most
 * applications should store MeshAttributeHandles instead of
 * MeshAttributeHandle, and after most of those changes are made we will
 * deprecate that name.
 */
template <typename T>
class MeshAttributeHandle : public TypedAttributeHandle<T>
{
public:
    friend class wmtk::Mesh;
    MeshAttributeHandle();
    MeshAttributeHandle(Mesh& m, const TypedAttributeHandle<T>&);
    MeshAttributeHandle(const MeshAttributeHandle<T>& o);
    MeshAttributeHandle(MeshAttributeHandle<T>&& o);
    MeshAttributeHandle<T>& operator=(const MeshAttributeHandle<T>& o);
    MeshAttributeHandle<T>& operator=(MeshAttributeHandle<T>&& o);


    const Mesh& mesh() const;
    Mesh& mesh();

    // creates mutable accessors
    // Implementations are in the MutableAccessor.hpp
    // for historical reasons note that the following two classes are the same:
    // wmtk::attribute::MutableAccessor
    // wmtk::Accessor
    MutableAccessor<T> create_accessor();

    // Creates const accessors
    // Implementations are in the ConstAccessor.hpp
    // for historical reasons note that the following two classes are the same:
    // wmtk::attribute::ConstAccessor
    // wmtk::ConstAccessor
    ConstAccessor<T> create_const_accessor() const;
    ConstAccessor<T> create_accessor() const;

private:
    Mesh* m_mesh = nullptr;
};
} // namespace wmtk::attribute


namespace wmtk {
template <typename T>
using MeshAttributeHandle = attribute::MeshAttributeHandle<T>;
} // namespace wmtk

