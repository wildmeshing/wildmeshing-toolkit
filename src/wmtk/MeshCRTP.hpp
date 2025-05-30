#pragma once
#include "Mesh.hpp"
#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
#include <ranges>
#endif


namespace wmtk {

namespace attribute {
template <typename T, typename MeshType, typename AttributeType, int Dim>
class Accessor;
}

/**
 * A [Curiously Recurring Template Pattern](https://en.cppreference.com/w/cpp/language/crtp) shim to enable generic specialization of functions.
 * CRTP allows us to shift from dynamic to static (inline-able) polymorphism for functions that are
 *frequently called (like id and switch_tuple). It also allows us to create slightly different
 *interfaces to return different types of accessors to take advantage of the static polymorphism.
 *
 **/
template <typename Derived>
class MeshCRTP : public Mesh
{
private:
    // using Mesh::create_accessor;
    // using Mesh::create_const_accessor;

public:
    template <typename U, typename MeshType, typename AttributeType, int Dim>
    friend class attribute::Accessor;
    template <int64_t cell_dimension, typename NodeFunctor>
    friend class multimesh::MultiMeshSimplexVisitor;
    using Mesh::Mesh;
    /// CRTP utility to extract the derived type of this
    Derived& derived() { return static_cast<Derived&>(*this); }
    /// CRTP utility to extract the derived type of this with constnesss
    const Derived& derived() const { return static_cast<const Derived&>(*this); }

    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override
    {
        assert(type <= top_simplex_type());
        return derived().switch_tuple(tuple, type);
    }
    /// Performs a sequence of switch_tuple operations in the order specified in op_sequence.
    /// in debug mode this will assert a failure, in release this will return a null tuple
#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
    template <std::ranges::forward_range ContainerType>
#else
    template <typename ContainerType>
#endif
    Tuple switch_tuples(const Tuple& tuple, const ContainerType& op_sequence) const;
    /// annoying initializer list prototype to catch switch_tuples(t, {PV,PE})
    Tuple switch_tuples(const Tuple& tuple, const std::initializer_list<PrimitiveType>& op_sequence)
        const;

    /// returns if a tuple is counterclockwise or not
    bool is_ccw(const Tuple& tuple) const override { return derived().is_ccw(tuple); }
    /// returns if a simplex is on the boundary of hte mesh. For anything but dimension - 1 this checks if this is the face of any boundary dimension-1 facet
    bool is_boundary(PrimitiveType pt, const Tuple& tuple) const override
    {
        return derived().is_boundary(pt, tuple);
    }

    /// constructs an accessor that is aware of the derived mesh's type
    template <typename T, int Dim = Eigen::Dynamic>
    inline attribute::Accessor<T, Derived, attribute::CachingAttribute<T>, Dim> create_accessor(
        const TypedAttributeHandle<T>& handle)
    {
        return attribute::Accessor<T, Derived, attribute::CachingAttribute<T>, Dim>(derived(), handle);
    }
    /// constructs a const accessor that is aware of the derived mesh's type
    template <typename T, int Dim = Eigen::Dynamic>
    const attribute::Accessor<T, Derived, attribute::CachingAttribute<T>, Dim> create_const_accessor(
        const attribute::TypedAttributeHandle<T>& handle) const
    {
        return attribute::Accessor<T, Derived, attribute::CachingAttribute<T>, Dim>(derived(), handle);
    }

    /// constructs a accessor that is aware of the derived mesh's type
    template <typename T, int Dim = Eigen::Dynamic>
    inline attribute::Accessor<T, Derived, attribute::CachingAttribute<T>, Dim> create_accessor(
        const attribute::MeshAttributeHandle& handle)
    {
        assert(&handle.mesh() == this);
        assert(handle.holds<T>());
        return create_accessor<T, Dim>(handle.as<T>());
    }


    /// constructs a const accessor that is aware of the derived mesh's type
    template <typename T, int Dim = Eigen::Dynamic>
    inline const attribute::Accessor<T, Derived, attribute::CachingAttribute<T>, Dim> create_const_accessor(
        const attribute::MeshAttributeHandle& handle) const
    {
        assert(&handle.mesh() == this);
        assert(handle.holds<T>());
        return create_const_accessor<T, Dim>(handle.as<T>());
    }

public:
    /// Returns the id of a simplex encoded in a tuple
    int64_t id(const Tuple& tuple, PrimitiveType type) const override
    {
        return derived().id(tuple, type);
    }

    /// variant of id that can cache internally held values
    int64_t id(const simplex::Simplex& s) const final override
    {
        return id(s.tuple(), s.primitive_type());
    }

    // catch any other Mesh id methods that might emerge by default
    using Mesh::id;

protected:
    /// internal utility for overriding the mesh class's id function without having the final override block the derived class's override
    /// (we can't have Mesh::id be virtual, MeshCRTP<Derived>::id final override, and TriMesh::id. This indirection pushes the final override to this other function
    int64_t id_virtual(const Tuple& tuple, PrimitiveType type) const final override
    {
        return id(tuple, type);
    }


protected:
    Tuple tuple_from_id(const PrimitiveType type, const int64_t gid) const override
    {
        return derived().tuple_from_id(type, gid);
    }
};

template <typename Derived>
#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
template <std::ranges::forward_range ContainerType>
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
