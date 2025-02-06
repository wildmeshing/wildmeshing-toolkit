#pragma once

#include <map>
#include <memory>
#include <wmtk/MeshBase.hpp>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/TypedAttributeHandle.hpp>

namespace wmtk {
namespace simplex {
class IdSimplex;
class Simplex;
} // namespace simplex
} // namespace wmtk

namespace wmtk::submesh {

class Embedding;

/**
 * Embedding m;
 * SubMesh s = m.add_sub_mesh();
 * s.add_simplex(IdSimplex);
 * s.add_from_tag(tag_handle, tag_value);
 */
class SubMesh : public std::enable_shared_from_this<SubMesh>, public MeshBase
{
public:
    SubMesh(Embedding& embedding, int64_t submesh_id);
    SubMesh() = delete;
    SubMesh(const SubMesh&) = delete;
    SubMesh& operator=(const SubMesh&) = delete;
    SubMesh(SubMesh&&) = delete;
    SubMesh& operator=(SubMesh&&) = delete;
    ~SubMesh() override = default;

    Mesh& mesh();
    const Mesh& mesh() const;

    /**
     * @brief Adds a simpex to the submesh.
     *
     * The top simplex type of the mesh is updated if the added simplex is of higher dimension than
     * any other simplex of the submesh.
     */
    void add_simplex(const Tuple& tuple, PrimitiveType pt);
    void add_simplex(const simplex::IdSimplex& simplex);

    std::vector<Tuple> get_all(const PrimitiveType pt) const override;

    /**
     * @brief Get the maximum primitive type that has a tag for a given tuple.
     */
    PrimitiveType top_simplex_type(const Tuple& tuple) const;

    /**
     * @brief Get the maximum primitive type that has a tag in the entire mesh.
     */
    PrimitiveType top_simplex_type() const;

    int64_t top_cell_dimension() const override;

    /**
     * @brief Can only perform local switches!
     *
     * Throws if `pt` is larger or equal to top_simplex_type(tuple)
     */
    Tuple switch_tuple(const Tuple& tuple, PrimitiveType pt) const override;

    /**
     * This function does not make sense. The proper way to navigate is using
     * cofaces_single_dimension.
     */
    // std::vector<Tuple> switch_tuple_vector(const Tuple& tuple, PrimitiveType pt) const;

    /**
     * @brief Is the given simplex on the boundary?
     *
     * This check is more complex than the one of the mesh itself but follows a similar idea. First,
     * the top simplex type in the open star is determined. Next, we check if any facet in the star
     * has less than two top simplices incident.
     *
     * Note that the behavior might be unexpected for non-homogenuous or non-manifold simplicial
     * complexes!
     */
    bool is_boundary(PrimitiveType pt, const Tuple& tuple) const override;
    bool is_boundary(const Tuple& tuple, PrimitiveType pt) const;

    // This is going to be some ugly recursive stuff I guess...
    bool is_manifold(PrimitiveType pt, const Tuple& tuple) const;

    // Check if sub mesh contains the simplex
    bool contains(const Tuple& tuple, PrimitiveType pt) const;
    bool contains(const simplex::IdSimplex& s) const;
    bool contains(const simplex::Simplex& s) const;

    template <typename T>
    void add_from_tag_attribute(
        const attribute::TypedAttributeHandle<T>& tag_attribute,
        const T tag_value);

    /**
     * Wrapping the simplex id function of Mesh.
     */
    int64_t id(const Tuple& tuple, PrimitiveType pt) const override;
    int64_t id(const simplex::Simplex& s) const override;

private:
    Embedding& m_embedding;
    const int64_t m_submesh_id;

    int64_t m_top_cell_dimension = -1;

private:
    Tuple local_switch_tuple(const Tuple& tuple, PrimitiveType pt) const;
};

} // namespace wmtk::submesh
