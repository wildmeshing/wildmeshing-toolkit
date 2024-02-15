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
    // Performs a sequence of switch_tuple operations in the order specified in op_sequence.
    // in debug mode this will assert a failure, in release this will return a null tuple
#if defined(__cpp_concepts)
    template <std::forward_iterator ContainerType>
#else
    template <typename ContainerType>
#endif
    Tuple switch_tuples(const Tuple& tuple, const ContainerType& op_sequence) const;
    // annoying initializer list prototype to catch switch_tuples(t, {PV,PE})
    Tuple switch_tuples(const Tuple& tuple, const std::initializer_list<PrimitiveType>& op_sequence)
        const;
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

template <typename Derived>
#if defined(__cpp_concepts)
template <std::forward_iterator ContainerType>
#else
template <typename ContainerType>
#endif
Tuple MeshCRTP<Derived>::switch_tuples(const Tuple& tuple, const ContainerType& sequence) const
{
    static_assert(std::is_same_v<typename ContainerType::value_type, PrimitiveType>);
    Tuple r = tuple;
    const PrimitiveType top_type = top_simplex_type();

    const int64_t boundary_dim = top_cell_dimension() - 1;
    const PrimitiveType boundary_pt = static_cast<PrimitiveType>(boundary_dim);

    for (const PrimitiveType primitive : sequence) {
        // for top level simplices we cannot navigate across boundaries
        if (primitive == top_type && is_boundary(boundary_pt, r)) {
            assert(!is_boundary(boundary_pt, r));
            r = {};
            return r;
        }
        r = switch_tuple(r, primitive);
    }
    return r;
}


template <typename Derived>
Tuple MeshCRTP<Derived>::switch_tuples(
    const Tuple& tuple,
    const std::initializer_list<PrimitiveType>& op_sequence) const
{
    return switch_tuples<std::initializer_list<PrimitiveType>>(tuple, op_sequence);
}
} // namespace wmtk
