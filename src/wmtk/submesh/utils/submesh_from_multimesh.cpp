#include "submesh_from_multimesh.hpp"

#include <wmtk/submesh/SubMesh.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::submesh::utils {

std::shared_ptr<Embedding> submesh_from_multimesh(const std::shared_ptr<Mesh>& mesh)
{
    // log_assert(mesh->is_multi_mesh_root(), "submesh_from_multimesh must be called on root mesh");

    std::shared_ptr<Embedding> emb_ptr = std::make_shared<Embedding>(mesh);
    Embedding& emb = *emb_ptr;

    for (const auto& child_mesh_ptr : mesh->get_child_meshes()) {
        auto sub_ptr = emb.add_submesh();

        const Mesh& cm = *child_mesh_ptr;
        for (const Tuple& child_tuple : cm.get_all(cm.top_simplex_type())) {
            const simplex::Simplex child_simplex(cm.top_simplex_type(), child_tuple);
            const simplex::Simplex parent_simplex = cm.map_to_parent(child_simplex);
            sub_ptr->add_simplex(parent_simplex);
        }
    }

    return emb_ptr;
}

} // namespace wmtk::submesh::utils