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
class MeshAttributeHandle
{
private:
    using HandleVariant = std::variant<
        TypedAttributeHandle<char>,
        TypedAttributeHandle<int64_t>,
        TypedAttributeHandle<double>,
        TypedAttributeHandle<Rational>>;

public:
    friend class wmtk::Mesh;
    friend class std::hash<MeshAttributeHandle>;
    MeshAttributeHandle();
    MeshAttributeHandle(Mesh& m, const HandleVariant&);
    MeshAttributeHandle(const MeshAttributeHandle& o);
    MeshAttributeHandle(MeshAttributeHandle&& o);
    MeshAttributeHandle<T>& operator=(const MeshAttributeHandle& o);
    MeshAttributeHandle<T>& operator=(MeshAttributeHandle&& o);

    template <typename U>
    bool operator==(const MeshAttributeHandle<U>& o) const
    {
        return TypedAttributeHandle<T>::operator==(o) && m_mesh == o.m_mesh;
    }


    void is_same_mesh(const Mesh&) const;


    bool is_valid() const { return TypedAttributeHandle<T>::is_valid() && m_mesh != nullptr; }


    template <typename T>
    const TypedAttributeHandle<T>& as() const
    {
        return std::get<TypedAttributeHandle<T>>(m_handle);
    }

private:
    Mesh* m_mesh = nullptr;
    HandleVariant m_handle;
};

} // namespace wmtk::attribute


namespace wmtk {
template <typename T>
using MeshAttributeHandle = attribute::MeshAttributeHandle<T>;
} // namespace wmtk
