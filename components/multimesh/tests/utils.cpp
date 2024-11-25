#include <wmtk/Mesh.hpp>
#include "tools/TriMesh_examples.hpp"
#include "utils.hpp"
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>

std::shared_ptr<wmtk::Mesh> make_mesh()
{
    return wmtk::tests::disk(5);
}

auto make_child(wmtk::Mesh& m, const std::vector<int64_t>& path)
    -> std::vector<std::shared_ptr<wmtk::Mesh>>
{
    if (path.size() == 0) {
        // multimesh root mesh already exists so nothing to be done
        return {};
    }
    std::vector<std::shared_ptr<wmtk::Mesh>> meshes;
    for (size_t j = 0; j < path.size(); ++j) {
        std::vector<int64_t> p(path.begin(), path.begin() + j);
        auto& cur_mesh = m.get_multi_mesh_mesh(p);
        int64_t child_index = path[j];
        const auto child_meshes = cur_mesh.get_child_meshes();
        for (int64_t index = child_meshes.size(); index <= child_index; ++index) {
            auto new_mesh = make_mesh();
            auto map = wmtk::multimesh::same_simplex_dimension_bijection(cur_mesh, *new_mesh);

            cur_mesh.register_child_mesh(new_mesh, map);
            meshes.emplace_back(new_mesh);
        }
    }
    return meshes;
}
