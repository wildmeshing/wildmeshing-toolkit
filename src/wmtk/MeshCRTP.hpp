#pragma once
#include "Mesh.hpp"


namespace wmtk {

namespace attribute {
template <typename T, typename MeshType>
class Accessor;
}

template <typename Derived>
class MeshCRTP : public Mesh
{
public:
    template <typename U, typename MeshType>
    friend class attribute::Accessor;
    using Mesh::Mesh;
    Derived& derived() { return static_cast<Derived&>(*this); }
    const Derived& derived() const { return static_cast<const Derived&>(*this); }

    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override
    {
        return derived().switch_tuple(tuple, type);
    }
    bool is_ccw(const Tuple& tuple) const override { return derived().is_ccw(tuple); }
    bool is_boundary(PrimitiveType pt, const Tuple& tuple) const override
    {
        return derived().is_boundary(pt, tuple);
    }

    template <typename T>
    inline attribute::Accessor<T, Derived> create_accessor(const TypedAttributeHandle<T>& handle)
    {
        return attribute::Accessor<T, Derived>(derived(), handle);
    }
    template <typename T>
    const attribute::Accessor<T, Derived> create_const_accessor(
        const TypedAttributeHandle<T>& handle) const
    {
        return attribute::Accessor<T, Derived>(derived(), handle);
    }

    template <typename T>
    attribute::Accessor<T, Derived> create_accessor(const attribute::MeshAttributeHandle& handle)
    {
        assert(&handle.mesh() == this);
        assert(handle.holds<T>());
        return create_accessor(handle.as<T>());
    }


    template <typename T>
    inline const attribute::Accessor<T, Derived> create_const_accessor(
        const attribute::MeshAttributeHandle& handle) const
    {
        assert(&handle.mesh() == this);
        assert(handle.holds<T>());
        return create_const_accessor(handle.as<T>());
    }

protected:
    int64_t id(const Tuple& tuple, PrimitiveType type) const { return derived().id(tuple, type); }
    int64_t id_virtual(const Tuple& tuple, PrimitiveType type) const final override
    {
        return id(tuple, type);
    }
    int64_t id(const simplex::Simplex& s) const final override
    {
        if (s.m_index == -1) {
            s.m_index = id(s.tuple(), s.primitive_type());
        }
        return s.m_index;
    }


protected:
    Tuple tuple_from_id(const PrimitiveType type, const int64_t gid) const override
    {
        return derived().tuple_from_id(type, gid);
    }
};
} // namespace wmtk
