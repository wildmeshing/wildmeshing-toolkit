#pragma once

#include <Eigen/Core>

#include <initializer_list>

#include <memory>
#include <tuple>


// need to return this header
#include "attribute/Accessor.hpp"
#include "attribute/internal/UseParentScopeRAII.hpp"

// basic data for the class
#include <wmtk/simplex/Simplex.hpp>
#include "MeshBase.hpp"
#include "Tuple.hpp"
#include "Types.hpp"
#include "attribute/AttributeManager.hpp"
#include "attribute/AttributeScopeHandle.hpp"
#include "attribute/FlagAccessor.hpp"
#include "attribute/MeshAttributeHandle.hpp"

#include "simplex/IdSimplex.hpp"
#include "simplex/NavigatableSimplex.hpp"
#include "simplex/Simplex.hpp"
#include "wmtk/attribute/CachingAttribute.hpp"


// if we have concepts then switch_tuples uses forward_iterator concept
#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
#include <ranges>
#endif


namespace wmtk {
namespace tests {
class DEBUG_Mesh;
} // namespace tests
// thread management tool that we will PImpl
namespace attribute {
class AttributeManager;
template <typename T, typename MeshType, typename AttributeType, int Dim>
class Accessor;

} // namespace attribute
namespace operations {
class Operation;
class EdgeCollapse;
class EdgeSplit;
class EdgeOperationData;
namespace internal {
class CollapseAlternateFacetData;
}
} // namespace operations

namespace io {
class ParaviewWriter;
}

namespace submesh {
class Embedding;
}


// NOTE: the implementation of this class is split into several files to improve clang-format
// performance
// * Mesh.cpp
// * Mesh_attributes.cpp
// * Mesh_construction.cpp
class Mesh : public std::enable_shared_from_this<Mesh>, public MeshBase
{
public:
    friend class tests::DEBUG_Mesh;
    template <typename T, typename MeshType, typename AttributeType, int Dim>
    friend class attribute::Accessor;
    friend class attribute::UseParentScopeRAII;
    friend class io::ParaviewWriter;
    friend class HDF5Reader;
    friend class attribute::AttributeManager;
    friend class submesh::Embedding;
    friend class operations::Operation;
    friend class operations::EdgeCollapse;
    friend class operations::EdgeSplit;
    friend class operations::EdgeOperationData;
    friend class operations::internal::CollapseAlternateFacetData;


    int64_t top_cell_dimension() const override;
    PrimitiveType top_simplex_type() const;
    bool is_free() const;

    MeshType mesh_type() const override;

    // dimension is the dimension of the top level simplex in this mesh
    // That is, a TriMesh is a 2, a TetMesh is a 3
    Mesh(const int64_t& dimension);
    // maximum primitive type id for supported attribute primitive locations
    Mesh(const int64_t& dimension, const int64_t& max_primitive_type_id);
    Mesh(Mesh&& other);
    Mesh(const Mesh& other) = delete;
    Mesh& operator=(const Mesh& other) = delete;
    Mesh& operator=(Mesh&& other);
    virtual ~Mesh();

    void serialize(MeshWriter& writer, const Mesh* local_root = nullptr) const;

    /**
     * Generate a vector of Tuples from global vertex/edge/triangle/tetrahedron index
     * @param type the type of tuple, can be vertex/edge/triangle/tetrahedron
     * @return vector of Tuples referring to each type
     */
    std::vector<Tuple> get_all(PrimitiveType type) const override;

    std::vector<simplex::IdSimplex> get_all_id_simplex(PrimitiveType type) const override;
    /**
     * @brief Retrieve the IdSimplex that is represented by the tuple and primitive type.
     */
    simplex::IdSimplex get_id_simplex(const Tuple& tuple, PrimitiveType pt) const;

    simplex::IdSimplex get_id_simplex(const simplex::Simplex& s) const;

    /**
     * @brief Convert an IdSimplex into a Simplex.
     */
    simplex::Simplex get_simplex(const simplex::IdSimplex& s) const;

    Tuple get_tuple_from_id_simplex(const simplex::IdSimplex& s) const;

    /**
     * Consolidate the attributes, moving all valid simplexes at the beginning of the corresponding
     * vector
     */
    virtual std::tuple<std::vector<std::vector<int64_t>>, std::vector<std::vector<int64_t>>>
    consolidate();

    /**
     * Returns a vector of vectors of attribute handles. The first index denotes the type of simplex
     * pointed by the attribute (i.e. the index type). As an example, the FV relationship points to
     * vertices so it should be returned in the slot [0].
     */
    virtual std::vector<std::vector<TypedAttributeHandle<int64_t>>> connectivity_attributes()
        const = 0;


    std::vector<attribute::MeshAttributeHandle::HandleVariant> builtin_attributes() const;
    std::vector<attribute::MeshAttributeHandle::HandleVariant> custom_attributes() const;


    /* @brief registers an attribute without assuming the mesh exists */
    template <typename T>
    [[nodiscard]] attribute::MeshAttributeHandle register_attribute(
        const std::string& name,
        PrimitiveType type,
        int64_t size,
        bool replace = false,
        T default_value = T(0));

    /* @brief registers an attribute without assuming the mesh exists, returns a typed attribute */
    template <typename T>
    [[nodiscard]] attribute::TypedAttributeHandle<T> register_attribute_typed(
        const std::string& name,
        PrimitiveType type,
        int64_t size,
        bool replace = false,
        T default_value = T(0));


public:
    template <typename T>
    bool has_attribute(
        const std::string& name,
        const PrimitiveType ptype) const; // block standard topology tools

    template <typename T>
    attribute::MeshAttributeHandle get_attribute_handle(
        const std::string& name,
        const PrimitiveType ptype) const; // block standard topology tools

    template <typename T>
    attribute::TypedAttributeHandle<T> get_attribute_handle_typed(
        const std::string& name,
        const PrimitiveType ptype) const; // block standard topology tools


    template <typename T, int D = Eigen::Dynamic>
    attribute::Accessor<T, Mesh, attribute::CachingAttribute<T>, D> create_accessor(
        const attribute::MeshAttributeHandle& handle);


    template <typename T, int D = Eigen::Dynamic>
    const attribute::Accessor<T, Mesh, attribute::CachingAttribute<T>, D> create_const_accessor(
        const attribute::MeshAttributeHandle& handle) const;

    template <typename T, int D = Eigen::Dynamic>
    attribute::Accessor<T, Mesh, attribute::CachingAttribute<T>, D> create_accessor(
        const TypedAttributeHandle<T>& handle);

    template <typename T, int D = Eigen::Dynamic>
    const attribute::Accessor<T, Mesh, attribute::CachingAttribute<T>, D> create_const_accessor(
        const TypedAttributeHandle<T>& handle) const;

    template <typename T>
    int64_t get_attribute_dimension(const TypedAttributeHandle<T>& handle) const;

    template <typename T>
    const T& get_attribute_default_value(const TypedAttributeHandle<T>& handle) const;

    template <typename T>
    std::string get_attribute_name(const TypedAttributeHandle<T>& handle) const;

    std::string get_attribute_name(
        const attribute::MeshAttributeHandle::HandleVariant& handle) const;

    /**
     * @brief Remove all custom attributes besides the one passed in.
     *
     * @param custom_attributes Vector of attributes that should be kept
     */
    void clear_attributes(
        const std::vector<attribute::MeshAttributeHandle::HandleVariant>& keep_attributes);
    void clear_attributes();
    void clear_attributes(const std::vector<attribute::MeshAttributeHandle>& keep_attributes);
    void delete_attribute(const attribute::MeshAttributeHandle& to_delete);
    void delete_attribute(const attribute::MeshAttributeHandle::HandleVariant& to_delete);


    // creates a scope as int64_t as the AttributeScopeHandle exists
    [[nodiscard]] attribute::AttributeScopeHandle create_scope();


    /**
     * @brief Evaluate the passed in function inside the parent scope.
     * The parent_scope function can be nested to reach deeper levels in the scope stack.
     *
     * @param f The function that is evaluated within the parent scope.
     * @param args... The other arguments to this function
     * @returns The return value of f.
     */
    template <typename Functor, typename... Args>
    decltype(auto) parent_scope(Functor&& f, Args&&... args) const;


    const attribute::FlagAccessor<Mesh> get_flag_accessor(PrimitiveType type) const;
    const attribute::FlagAccessor<Mesh> get_const_flag_accessor(PrimitiveType type) const;


    bool operator==(const Mesh& other) const;

    void assert_capacity_valid() const;
    virtual bool is_connectivity_valid() const = 0;

    virtual std::vector<Tuple> orient_vertices(const Tuple& t) const = 0;

protected: // member functions
    attribute::FlagAccessor<> get_flag_accessor(PrimitiveType type);


protected:
    /**
     * @brief internal function that returns the tuple of requested type, and has the global index
     * cid
     *
     * @param gid
     * @return Tuple
     */
    virtual Tuple tuple_from_id(const PrimitiveType type, const int64_t gid) const = 0;
    simplex::NavigatableSimplex simplex_from_id(const PrimitiveType type, const int64_t gid) const;
    std::vector<std::vector<int64_t>> simplices_to_gids(
        const std::vector<std::vector<simplex::Simplex>>& simplices) const;
    /**
     * @brief reserve space for all attributes data types for all dimensional simplices
     *
     * @param top_d the top dimensional simplex
     */
    void reserve_attributes_to_fit();
    void reserve_attributes(PrimitiveType type, int64_t size);
    void reserve_attributes(int64_t dimension, int64_t size);


    // specifies the number of simplices of each type and resizes attributes appropritely
    void set_capacities(std::vector<int64_t> capacities);

    // reserves extra attributes than necessary right now
    void reserve_more_attributes(PrimitiveType type, int64_t size);
    // reserves extra attributes than necessary right now, does not pay attention
    void reserve_more_attributes(const std::vector<int64_t>& sizes);

    // makes sure that there are at least `size` simples of type `type` avialable
    void guarantee_more_attributes(PrimitiveType type, int64_t size);
    // makes sure that there are at least `size` simplices avialable at every dimension
    void guarantee_more_attributes(const std::vector<int64_t>& sizes);

    // makes sure that there are at least `size` simples of type `type` avialable
    void guarantee_at_least_attributes(PrimitiveType type, int64_t size);
    // makes sure that there are at least `size` simplices avialable at every dimension
    void guarantee_at_least_attributes(const std::vector<int64_t>& sizes);

    // provides new simplices - should ONLY be called in our atomic topological operations
    // all returned simplices are active (i.e their flags say they exist)
    [[nodiscard]] std::vector<int64_t> request_simplex_indices(PrimitiveType type, int64_t count);

public:
    /**
     * @brief switch the orientation of the Tuple of the given dimension
     * @note this is not done in place. Return a new Tuple of the switched state
     *
     * @param m
     * @param type  d-0 -> switch vertex
                    d-1 -> switch edge
                    d-2 -> switch face
                    d-3 -> switch tetrahedron
    */
    virtual Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override = 0;

    // NOTE: adding per-simplex utility functions here is _wrong_ and will be removed


    // Performs a sequence of switch_tuple operations in the order specified in op_sequence.
    // in debug mode this will assert a failure, in release this will return a null tuple
#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
    template <std::ranges::forward_range ContainerType>
#else
    template <typename ContainerType>
#endif
    Tuple switch_tuples(const Tuple& tuple, const ContainerType& op_sequence) const;
    // annoying initializer list prototype to catch switch_tuples(t, {PV,PE})
    Tuple switch_tuples(const Tuple& tuple, const std::initializer_list<PrimitiveType>& op_sequence)
        const;

    // Performs a sequence of switch_tuple operations in the order specified in op_sequence.
#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
    template <std::ranges::forward_range ContainerType>
#else
    template <typename ContainerType>
#endif
    Tuple switch_tuples_unsafe(const Tuple& tuple, const ContainerType& op_sequence) const;
    // annoying initializer list prototype to catch switch_tuples(t, {PV,PE})
    Tuple switch_tuples_unsafe(
        const Tuple& tuple,
        const std::initializer_list<PrimitiveType>& op_sequence) const;

    void set_capacities_from_flags();
    /**
     * @brief read in the m_capacities return the upper bound for the number of entities of the
     * given dimension
     *
     * @param type
     * @return int
     */
    int64_t capacity(PrimitiveType type) const;

    /**
     * @brief TODO this needs dimension?
     *
     * @param m
     * @return true if the Tuple is oriented counter-clockwise
     * @return false
     */
    virtual bool is_ccw(const Tuple& tuple) const = 0;

    /**
     * @brief check if a simplex lies on a boundary or not
     *
     * @param simplex
     * @return true if this simplex lies on the boundary of the mesh
     * @return false otherwise
     */
    bool is_boundary(const simplex::Simplex& tuple) const;
    /**
     * @brief check if a simplex (encoded as a tuple/primitive pair) lies on a boundary or not
     *
     * @param simplex
     * @return true if this simplex lies on the boundary of the mesh
     * @return false otherwise
     */
    virtual bool is_boundary(PrimitiveType, const Tuple& tuple) const override = 0;


    bool is_hash_valid(const Tuple& tuple, const attribute::Accessor<int64_t>& hash_accessor) const;
    bool is_hash_valid(const Tuple& tuple) const;

    /**
     * @brief check validity of tuple including its hash
     *
     * @param tuple the tuple to be checked
     * @param type only the top cell dimension, other validity follows with assumption of
     * manifoldness. 2->triangle, 3->tetrahedron
     * @return true if is valid
     * @return false
     */
    virtual bool is_valid(const Tuple& tuple) const;

    // whether the tuple refers to a removed / invalid dart in the mesh
    bool is_removed(const Tuple& tuple) const;
    // whether the tuple refers to a removed / invalid simplex in the mesh
    bool is_removed(const Tuple& tuple, PrimitiveType pt) const;

protected:
    // whether the tuple refers to a removed / invalid facet
    bool is_removed(int64_t index) const;
    // whether the tuple refers to a removed / invalid simplex
    bool is_removed(int64_t index, PrimitiveType pt) const;

public:
    /**
     * @brief Check if the cached id in a simplex is up-to-date.
     */
    bool is_valid(const simplex::Simplex& s) const;

    template <typename T>
    bool validate_handle(const TypedAttributeHandle<T>& handle) const;

    bool has_embedding() const;

    submesh::Embedding& get_embedding() const;

protected:
    // creates a scope as int64_t as the AttributeScopeHandle exists
    [[nodiscard]] attribute::AttributeScopeHandle create_single_mesh_scope();

public:
    /**
     * @brief return the global id of the Tuple of the given dimension
     *
     * This function uses the implementation defined in a derived class (like a Tri/TetMesh) using a virtual function, but to enable more opportunities teh actual virtual function is id_virtual
     *
     * @param m
     * @param type  d-0 -> vertex
                    d-1 -> edge
                    d-2 -> face
                    d-3 -> tetrahedron
        * @return int64_t id of the entity
    */
    int64_t id(const Tuple& tuple, PrimitiveType type) const override;

    int64_t id(const simplex::NavigatableSimplex& s) const { return s.index(); }
    int64_t id(const simplex::IdSimplex& s) const { return s.index(); }

    /// Forwarding version of id on simplices that does id caching
    virtual int64_t id(const simplex::Simplex& s) const override = 0;

protected:
    /// Internal utility to allow id to be virtual with a non-virtual overload in derived -Mesh classes.
    /// Mesh::id invokes Mesh::id_virtual which is final overriden by MeshCRTP<TriMesh>::id_virtual, which in turn invokes MeshCRTP<TriMesh>::id, and then TriMesh::id.
    /// This circuitous mechanism makes MeshCRTP<TriMesh>::id and TriMesh::id fully inlineable, so code that wants to take in any derived class can get optimized results with MeshCRTP, or for cases where classes want to utilize just TriMesh they can get inline/accelerated usage as well.
    virtual int64_t id_virtual(const Tuple& tuple, PrimitiveType type) const = 0;


    template <typename T, typename MeshType>
    static auto& get_index_access(attribute::Accessor<T, MeshType>& attr)
    {
        return attr.attribute();
    }
    template <typename T, typename MeshType>
    static auto& get_index_access(const attribute::Accessor<T, MeshType>& attr)
    {
        return attr.attribute();
    }


    // std::shared_ptr<AccessorCache> request_accesor_cache();
    //[[nodiscard]] AccessorScopeHandle push_accesor_scope();

protected: // THese are protected so unit tests can access - do not use manually in other derived
           // classes?
    attribute::AttributeManager m_attribute_manager;

    submesh::Embedding* m_embedding = nullptr; // a pointer to the embedding (if there is any)

    int64_t m_top_cell_dimension = -1;

    // assumes no adjacency data exists
    bool m_is_free = false;

private:
    // PImpl'd manager of per-thread update stacks
    // Every time a new access scope is requested the manager creates another level of indirection
    // for updates
    // std::unique_ptr<AttributeScopeManager> m_attribute_scope_manager;

    //=========================================================
    // simplex::Simplex Attribute
    //=========================================================


    /**
     * @brief   0x1 == true = simplex is active (simplex exists)
     *          all flag default to 0
     *
     */
    std::vector<TypedAttributeHandle<char>> m_flag_handles;


    /**
     * Generate a vector of Tuples from global vertex/edge/triangle/tetrahedron index
     * @param type the type of tuple, can be vertex/edge/triangle/tetrahedron
     * @param include_deleted if true returns also the deleted tuples (default false)
     * @return vector of Tuples referring to each type
     */
    std::vector<Tuple> get_all(PrimitiveType type, const bool include_deleted) const;
    std::vector<simplex::IdSimplex> get_all_id_simplex(
        PrimitiveType type,
        const bool include_deleted) const;
};


template <typename T, int D>
inline auto Mesh::create_accessor(const TypedAttributeHandle<T>& handle)
    -> attribute::Accessor<T, Mesh, attribute::CachingAttribute<T>, D>
{
    return attribute::Accessor<T, Mesh, attribute::CachingAttribute<T>, D>(*this, handle);
}
template <typename T, int D>
inline auto Mesh::create_const_accessor(const TypedAttributeHandle<T>& handle) const
    -> const attribute::Accessor<T, Mesh, attribute::CachingAttribute<T>, D>
{
    return attribute::Accessor<T, Mesh, attribute::CachingAttribute<T>, D>(*this, handle);
}

template <typename T, int D>
inline auto Mesh::create_accessor(const attribute::MeshAttributeHandle& handle)
    -> attribute::Accessor<T, Mesh, attribute::CachingAttribute<T>, D>
{
    assert(&handle.mesh() == this);
    assert(handle.holds<T>());
    return create_accessor<T, D>(handle.as<T>());
}


template <typename T, int D>
inline auto Mesh::create_const_accessor(const attribute::MeshAttributeHandle& handle) const
    -> const attribute::Accessor<T, Mesh, attribute::CachingAttribute<T>, D>
{
    assert(&handle.mesh() == this);
    assert(handle.holds<T>());
    return create_const_accessor<T, D>(handle.as<T>());
}

template <typename T>
inline attribute::MeshAttributeHandle Mesh::get_attribute_handle(
    const std::string& name,
    const PrimitiveType ptype) const
{
    return wmtk::attribute::MeshAttributeHandle(
        *const_cast<Mesh*>(this),
        get_attribute_handle_typed<T>(name, ptype));
}

template <typename T>
inline attribute::TypedAttributeHandle<T> Mesh::get_attribute_handle_typed(
    const std::string& name,
    const PrimitiveType ptype) const
{
    wmtk::attribute::TypedAttributeHandle<T> h;
    h.m_index = m_attribute_manager.get<T>(ptype).attribute_handle(name);
    h.m_primitive_type = ptype;
    return h;
}
template <typename T>
inline bool Mesh::has_attribute(const std::string& name, const PrimitiveType ptype) const
{
    return m_attribute_manager.get<T>(ptype).has_attribute(name);
}

template <typename T>
inline int64_t Mesh::get_attribute_dimension(const TypedAttributeHandle<T>& handle) const
{
    return m_attribute_manager.get_attribute_dimension(handle);
}

template <typename T>
inline std::string Mesh::get_attribute_name(const TypedAttributeHandle<T>& handle) const
{
    return m_attribute_manager.get_name(handle);
}

template <typename Functor, typename... Args>
inline decltype(auto) Mesh::parent_scope(Functor&& f, Args&&... args) const
{
    attribute::UseParentScopeRAII raii(const_cast<Mesh&>(*this));
    return std::invoke(std::forward<Functor>(f), std::forward<Args>(args)...);
}

#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
template <std::ranges::forward_range ContainerType>
#else
template <typename ContainerType>
#endif
inline Tuple Mesh::switch_tuples(const Tuple& tuple, const ContainerType& sequence) const
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
inline bool Mesh::is_free() const
{
    return m_is_free;
}

inline MeshType Mesh::mesh_type() const
{
    return MeshType::Mesh;
}

inline int64_t Mesh::top_cell_dimension() const
{
    return m_top_cell_dimension;
}
inline PrimitiveType Mesh::top_simplex_type() const
{
    int64_t dimension = top_cell_dimension();
    assert(dimension >= 0);
    assert(dimension < 4);
    return static_cast<PrimitiveType>(dimension);
}


#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
template <std::ranges::forward_range ContainerType>
#else
template <typename ContainerType>
#endif
inline Tuple Mesh::switch_tuples_unsafe(const Tuple& tuple, const ContainerType& sequence) const
{
    static_assert(std::is_same_v<typename ContainerType::value_type, PrimitiveType>);
    Tuple r = tuple;
    for (const PrimitiveType primitive : sequence) {
        r = switch_tuple(r, primitive);
    }
    return r;
}

template <typename T>
inline bool Mesh::validate_handle(const TypedAttributeHandle<T>& handle) const
{
    return m_attribute_manager.validate_handle(handle);
}

inline int64_t Mesh::id(const Tuple& tuple, PrimitiveType type) const
{
    return id_virtual(tuple, type);
}
} // namespace wmtk
