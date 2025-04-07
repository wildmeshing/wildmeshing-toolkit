#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components::simplicial_embedding::internal {

/*
 * This class is used to seperate mesh and make sure there are no direct connection
 * between independent simplicity collection
 */
class SimplicialEmbedding
{
public:
    SimplicialEmbedding(
        Mesh& mesh,
        const std::vector<attribute::MeshAttributeHandle>& label_attributes,
        const int64_t& value,
        const std::vector<attribute::MeshAttributeHandle>& pass_through_attributes);

    /**
     * @brief Regularize tags in mesh.
     *
     * First, it is made sure that tags represent a simplicial complex, i.e., all faces of a tagged
     * simplex are also tagged.
     * Second (optional), split simplices who's faces are tagged but that are not tagged themselves.
     */
    void regularize_tags(bool generate_simplicial_embedding = true);

private:
    Mesh& m_mesh;

    std::vector<attribute::MeshAttributeHandle> m_label_attributes;
    int64_t m_value;

    std::vector<attribute::MeshAttributeHandle> m_pass_through_attributes;
};

} // namespace wmtk::components::simplicial_embedding::internal