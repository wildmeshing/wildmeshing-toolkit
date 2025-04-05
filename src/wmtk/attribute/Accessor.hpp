#pragma once
#include <algorithm>
#include <wmtk/simplex/IdSimplex.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include "CachingAttribute.hpp"
#include "TypedAttributeHandle.hpp"

namespace wmtk {
class HDF5Reader;
}
namespace wmtk::tests {
class DEBUG_PointMesh;
class DEBUG_EdgeMesh;
class DEBUG_TriMesh;
} // namespace wmtk::tests
namespace wmtk::tests_3d {
class DEBUG_TetMesh;
}

namespace wmtk::attribute {
class MeshAttributeHandle;

template <typename MeshType>
class IndexFlagAccessor;

/**
 * An Accessor that uses tuples for accessing attributes instead of indices.
 * The parameter T is the type of the attribute data, and always must match the underlying
 * attribute's type MeshType can default to Mesh, but specializing it with the specific type of mesh
 * this attribute lies in avoids some virtual calls AttributeType_ specifies the type of teh
 * underlying attribute, which should almost always be CachingAttribute<T> to take advantage of our
 * transactional caching system, but for unit tests (or guaranteed raw access to the underlying
 * buffers) this can be changed to Attribute<T> Dim specifies the dimension of the underlying
 * attribute - specifying this parameter when possible can have a notable performance improvement,
 * so specify it when possible. This performance change is the change between passing a single
 * pointer for each attribute vs a pointer + int size for each value
 */
template <
    typename T,
    typename MeshType = Mesh,
    typename AttributeType_ = CachingAttribute<T>,
    int Dim = Eigen::Dynamic>
class Accessor
{
public:
    friend class wmtk::Mesh;
    friend class wmtk::tests::DEBUG_PointMesh;
    friend class wmtk::tests::DEBUG_EdgeMesh;
    friend class wmtk::tests::DEBUG_TriMesh;
    friend class wmtk::tests_3d::DEBUG_TetMesh;
    template <typename MeshType>
    friend class IndexFlagAccessor;
    friend class HDF5Reader;

    using Scalar = T;

    using AttributeType = AttributeType_; // CachingAttribute<T>;

    // Even though the attribute dimension is specified above by the Dim parameter, we still allow
    // users to specify the dimension of the attributes they extract - this lets Dim=Eigen::Dynamic
    // accessors be passed around with internal logic specifying the appropriate dimension
    // Eigen::Map<Vector<T,D>>
    template <int D = Dim>
    using MapResult = typename AttributeType::template MapResult<D>;
    // Eigen::Map<const Vector<T,D>>
    template <int D = Dim>
    using ConstMapResult = typename AttributeType::template ConstMapResult<D>;


    Accessor(MeshType& m, const TypedAttributeHandle<T>& handle);
    Accessor(const MeshType& m, const TypedAttributeHandle<T>& handle);

    template <typename OMType, typename OAT, int D>
    Accessor(const Accessor<T, OMType, OAT, D>& o);

    // dimension of the attribute (the return types are a dimensino-vector)
    auto dimension() const -> int64_t;
    // the number of (vector-)values that can be written to. Note this is can be larger than the
    // size of the mesh
    auto reserved_size() const -> int64_t;
    // the default value assigned to new values of this attribute (for vector valued attributes
    // every dimension holds is this)
    auto default_value() const -> const Scalar&;


    // =============
    // Access methods
    // =============
    // NOTE: If any compilation warnings occur check that there is an overload for an index method


    // Access to a vector-valued attribute
    /**
     * @brief Access function for a vector attribute.
     *
     * @param t Tuple, Simplex, or IdSimplex for which the attribute should be accessed.
     * @return A reference (some Eigen::Map) to the attribute value.
     */
    template <int D = Dim, typename ArgType = wmtk::Tuple>
    MapResult<std::max(D, Dim)> vector_attribute(const ArgType& t);
    /**
     * @brief Constant access function for a vector attribute.
     *
     * @param t Tuple, Simplex, or IdSimplex for which the attribute should be accessed.
     * @return A const reference (some Eigen::Map) to the attribute value.
     */
    template <int D = Dim, typename ArgType = wmtk::Tuple>
    ConstMapResult<std::max(D, Dim)> const_vector_attribute(const ArgType& t) const;

    /**
     * @brief Access function for a scalar attribute.
     *
     * The scalar attribute circumvents the Eigen::Map that is necessary for vector attributes.
     *
     * @param t Tuple, Simplex, or IdSimplex for which the attribute should be accessed.
     * @return A reference to the attribute value.
     */
    template <typename ArgType>
    Scalar& scalar_attribute(const ArgType& t);
    /**
     * @brief Constant access function for a scalar attribute.
     *
     * The scalar attribute circumvents the Eigen::Map that is necessary for vector attributes.
     *
     * @param t Tuple, Simplex, or IdSimplex for which the attribute should be accessed.
     * @return A const reference to the attribute value.
     */
    template <typename ArgType>
    const Scalar& const_scalar_attribute(const ArgType& t) const;

    // For topological attributes we want to select teh value (global_id) by its (local_id) index,
    // for whichever local index this attribute lies on For instance, to get the global index of a
    // vertex from a FV matrix we want to access const_vector_attribute(global_id)(local_vid). This
    // function lets us use the primitive type to identify which local id we want.
    const T& const_topological_scalar_attribute(const Tuple& t, PrimitiveType pt) const;


    MeshType& mesh();
    const MeshType& mesh() const;

    int64_t transaction_depth() const;

protected:
    /**
     * @brief Retrieve the global ID of the given simplex.
     *
     * The simplex is defined by the given Tuple `t` and the PrimitiveType of the attribute.
     *
     * @param t Tuple used for accessing.
     * @return Global simplex ID.
     */
    int64_t index(const Tuple& t) const;
    // Simplex's primitive type must match this attribute, should assert failure if not
    /**
     * @brief Retrieve the global ID of the given simplex.
     *
     * Simplex's primitive type must match this attribute, should assert failure if not.
     *
     * @param t Simplex for accessing.
     * @return Global simplex ID.
     */
    int64_t index(const simplex::Simplex& t) const;
    /**
     * @brief Same as with `Simplex`.
     */
    int64_t index(const simplex::IdSimplex& t) const;

    int64_t index(const int64_t t) const;

    // The underlying attribute object.
    AttributeType& attribute();
    // The underlying attribute object
    const AttributeType& attribute() const;

public:
    MeshAttributeHandle handle() const;
    const TypedAttributeHandle<T>& typed_handle() const;
    PrimitiveType primitive_type() const;

private:
    TypedAttributeHandle<T> m_handle;
    MeshType& m_mesh;
    AttributeType& m_attribute;
};
} // namespace wmtk::attribute
#include "Accessor.hxx"
