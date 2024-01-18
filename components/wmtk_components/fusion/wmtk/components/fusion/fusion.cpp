#include "fusion.hpp"
#include "FusionOptions.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>


namespace wmtk::components {

void fusion(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    // load mesh
    FusionOptions options = j.get<FusionOptions>();
    auto mesh = cache.read_mesh(options.input);

    // get fusion axis
    int64_t fusion_axis = static_cast<int64_t>(options.fusion_axis);

    // get mesh dimension and checks
    int64_t mesh_dim = mesh->top_cell_dimension();

    auto pos_handle = mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos_accessor = mesh->create_accessor<double>(pos_handle.as<double>());
    double eps = 1e-8;

    switch (mesh_dim) {
    case (2): {
        if (fusion_axis == 2) {
            throw std::runtime_error("cannot fusion axis Z of a 2D mesh");
        }

        std::map<int64_t, int64_t> vertex_map;
        const auto& vertices = mesh->get_all(PrimitiveType::Vertex);

        std::array<std::vector<int64_t>, 2> vertices_on_zero;
        std::array<std::vector<int64_t>, 2> vertices_on_one;

        for (int64_t i = 0; i < vertices.size(); ++i) {
            const auto& v = vertices[i];
            const auto& pos = pos_accessor.vector_attribute(v);

            if (abs(pos[0] - 0.0) < eps) {
                vertices_on_zero[0].push_back(i);
            }
            if (abs(pos[1] - 0.0) < eps) {
                vertices_on_zero[1].push_back(i);
            }
            if (abs(pos[0] - 1.0) < eps) {
                vertices_on_one[0].push_back(i);
            }
            if (abs(pos[1] - 1.0) < eps) {
                vertices_on_one[1].push_back(i);
            }
        }

        // merge vertices
        for (int axis = 0; axis < 2; ++axis) {
            if (axis != fusion_axis && fusion_axis != 3) continue;
        }


        break;
    }
    case (3): {
        break;
    }
    default: {
        throw std::runtime_error("mesh dimension not supported");
    }
    }
}
} // namespace wmtk::components