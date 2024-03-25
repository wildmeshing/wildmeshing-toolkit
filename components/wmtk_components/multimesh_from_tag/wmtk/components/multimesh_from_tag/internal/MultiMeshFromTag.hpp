#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components::internal {

/**
 * @brief This class generates a multi-mesh from a mesh where the substructure is represented by a
 * tag.
 */
class MultiMeshFromTag
{
public:
    MultiMeshFromTag(
        Mesh& mesh,
        const attribute::MeshAttributeHandle& tag_handle,
        const int64_t tag_value);


    Eigen::MatrixX<int64_t> get_new_id_matrix(const PrimitiveType ptype) const;
    VectorXl get_new_top_coface_vector(const PrimitiveType ptype) const;

    Eigen::MatrixX<int64_t> adjacency_matrix() const;

    /**
     * @brief Returns a pointer to the substructure soup.
     *
     * The soup is merely an intermediate representation and should not be used. The function
     * `substructure()` is most likely what you are looking for.
     */
    std::shared_ptr<Mesh> substructure_soup() const;

    /**
     * @brief Returns a pointer to the manifold substructure mesh.
     *
     * This mesh is the substructure that is represented by tags in the input mesh. Any non-manifold
     * simplices were duplicated to make the substructure manifold. Besides that, no topological
     * changes were performed.
     */
    std::shared_ptr<Mesh> substructure() const;

    /**
     * @brief Create a manifold mesh from the substructure.
     */
    void compute_substructure_mesh();

    bool is_substructure_simplex_manifold(const simplex::Simplex& s) const;

    bool is_root_simplex_manifold(const simplex::Simplex& s) const;

    /**
     * @brief Remove the substructure soup from the multimesh.
     *
     * The substructure soup is used as an intermediate representation to construct the manifold
     * substructure mesh. This method removes the soup from the multimesh and is therefore necessary
     * to get a clean multimesh.
     */
    void remove_soup();

private:
    /**
     * @brief Compute the ids of the manifold substructure.
     *
     * This function utilizes the substructure soup to store the ids.
     */
    void compute_substructure_ids();

    /**
     * @brief Create a multimesh where the child-mesh is just a soup (no connectivity) of m_mesh.
     *
     * The soup is required as an intermediate step to construct the proper manifold substructure.
     */
    void create_substructure_soup();

    /**
     * @brief Get tuples with different global_cid that all represent simplex(t_in, ptype) and are
     * in the same tag-region.
     *
     * To find the tag-region a breadth-first-search is utilized where it is not allowed to cross
     * non-manifold d-1-simplices.
     *
     */
    std::vector<Tuple> get_connected_region(const Tuple& t, const PrimitiveType ptype);

    /**
     * Create the adjacency (stored as attribute) of the substructure
     */
    void build_adjacency_matrix();

private:
    Mesh& m_mesh;

    const attribute::Accessor<int64_t> m_tag_acc;
    const int64_t m_tag_value;
    const PrimitiveType m_tag_ptype;

    std::map<PrimitiveType, attribute::MeshAttributeHandle> m_new_id_handles;

    attribute::MeshAttributeHandle m_adjacency_handle;
    Eigen::MatrixX<int64_t> m_adjacency_matrix;
    std::map<PrimitiveType, Eigen::MatrixX<int64_t>> m_new_id_matrices;
    std::map<PrimitiveType, VectorXl> m_new_top_coface_vectors;
    std::shared_ptr<Mesh> m_soup_ptr;
    std::shared_ptr<Mesh> m_substructure_ptr;
};

} // namespace wmtk::components::internal
