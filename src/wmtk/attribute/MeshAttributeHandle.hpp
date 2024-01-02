#pragma once
#include "TypedAttributeHandle.hpp"

#include <variant>

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
    using Type = T;

    friend class wmtk::Mesh;
    friend class std::hash<MeshAttributeHandle<T>>;
    MeshAttributeHandle();
    MeshAttributeHandle(Mesh& m, const TypedAttributeHandle<T>&);
    MeshAttributeHandle(Mesh& m, const TypedAttributeHandleVariant&);
    MeshAttributeHandle(const MeshAttributeHandle<T>& o);
    MeshAttributeHandle(MeshAttributeHandle<T>&& o);
    MeshAttributeHandle<T>& operator=(const MeshAttributeHandle<T>& o);
    MeshAttributeHandle<T>& operator=(MeshAttributeHandle<T>&& o);

    template <typename U>
    bool operator==(const MeshAttributeHandle<U>& o) const
    {
        return TypedAttributeHandle<T>::operator==(o) && m_mesh == o.m_mesh;
    }


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

    // return the dimension of the attribute (i.e the number of values stored per simplex)
    int64_t dimension() const;

    std::string name() const;

    bool is_valid() const { return TypedAttributeHandle<T>::is_valid() && m_mesh != nullptr; }

private:
    Mesh* m_mesh = nullptr;
};

using MeshAttributeHandleVariant = std::variant<
    MeshAttributeHandle<char>,
    MeshAttributeHandle<int64_t>,
    MeshAttributeHandle<double>,
    MeshAttributeHandle<Rational>>;
} // namespace wmtk::attribute


namespace wmtk {
template <typename T>
using MeshAttributeHandle = attribute::MeshAttributeHandle<T>;
} // namespace wmtk
