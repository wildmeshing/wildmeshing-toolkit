#include "fusion.hpp"
#include "FusionOptions.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components {

void fusion(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    // load mesh
    FusionOptions options = j.get<FusionOptions>();
    auto mesh = cache.read_mesh(options.input);

    // get fusion axis
    int64_t fusion_axis = static_cast<int64_t>(options.fusion_axis);
    // int64_t fusion_axis = options.fusion_axis;


    // get mesh dimension and checks
    int64_t mesh_dim = mesh->top_cell_dimension();

    auto pos_handle = mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    const auto pos_accessor = mesh->create_const_accessor<double>(pos_handle.as<double>());
    double eps = 1e-8;

    switch (mesh_dim) {
    case (2): {
        if (fusion_axis == 2) {
            throw std::runtime_error("cannot fusion axis Z of a 2D mesh");
        }

        std::map<int64_t, int64_t> vertex_map;
        const auto& vertices = mesh->get_all(PrimitiveType::Vertex);

        std::array<std::vector<std::pair<int64_t, Eigen::VectorXd>>, 2> vertices_on_zero;
        std::array<std::vector<std::pair<int64_t, Eigen::VectorXd>>, 2> vertices_on_one;

        int64_t v00 = -1;
        int64_t v01 = -1;
        int64_t v10 = -1;
        int64_t v11 = -1;

        for (int64_t i = 0; i < vertices.size(); ++i) {
            const auto& v = vertices[i];
            const auto& pos = pos_accessor.const_vector_attribute(v);

            if (abs(pos[0] - 0.0) < eps) {
                vertices_on_zero[0].push_back(std::make_pair(i, pos));
            }
            if (abs(pos[1] - 0.0) < eps) {
                vertices_on_zero[1].push_back(std::make_pair(i, pos));
            }
            if (abs(pos[0] - 1.0) < eps) {
                vertices_on_one[0].push_back(std::make_pair(i, pos));
            }
            if (abs(pos[1] - 1.0) < eps) {
                vertices_on_one[1].push_back(std::make_pair(i, pos));
            }

            // corners
            if (abs(pos[0] - 0.0) < eps && abs(pos[1] - 0.0) < eps) {
                if (v00 != -1) throw std::runtime_error("More than 1 vertices on corner 00.");
                v00 = i;
            }
            if (abs(pos[0] - 0.0) < eps && abs(pos[1] - 1.0) < eps) {
                if (v01 != -1) throw std::runtime_error("More than 1 vertices on corner 01.");
                v01 = i;
            }
            if (abs(pos[0] - 1.0) < eps && abs(pos[1] - 0.0) < eps) {
                if (v10 != -1) throw std::runtime_error("More than 1 vertices on corner 10.");
                v10 = i;
            }
            if (abs(pos[0] - 1.0) < eps && abs(pos[1] - 1.0) < eps) {
                if (v11 != -1) throw std::runtime_error("More than 1 vertices on corner 11.");
                v11 = i;
            }
        }

        // merge vertices
        for (int axis = 0; axis < 2; ++axis) {
            if (axis != fusion_axis && fusion_axis != 3) continue;

            if (vertices_on_zero[axis].size() == vertices_on_one[axis].size()) {
                throw std::runtime_error("vertices size on the fusion axis does not match!");
            }

            auto cmp = [&](const std::pair<int64_t, Eigen::VectorXd>& a,
                           const std::pair<int64_t, Eigen::VectorXd>& b) {
                return a.second[1 - axis] < b.second[1 - axis];
            };

            std::sort(vertices_on_zero[axis].begin(), vertices_on_zero[axis].end(), cmp);
            std::sort(vertices_on_one[axis].begin(), vertices_on_one[axis].end(), cmp);

            for (int64_t i = 0; i < vertices_on_zero[axis].size(); ++i) {
                assert(
                    abs(vertices_on_zero[axis][i].second[1 - axis] -
                        vertices_on_one[axis][i].second[1 - axis]) < eps);

                vertex_map[vertices_on_one[axis][i].first] = vertices_on_zero[axis][i].first;
            }
        }

        // special case for fusion all axis
        if (fusion_axis == 3) {
            vertex_map[v00] = v00;
            vertex_map[v01] = v00;
            vertex_map[v10] = v00;
            vertex_map[v11] = v00;
        }

        // create periodic mesh
        RowVectors3l FV;
        RowVectors3d V(vertices.size(), 3);

        for (int64_t i = 0; i < vertices.size(); ++i) {
            V.row(i) = pos_accessor.const_vector_attribute(vertices[i]);
        }

        const auto& faces = mesh->get_all(PrimitiveType::Face);
        FV.resize(faces.size(), 3);

        for (int64_t i = 0; i < faces.size(); ++i) {
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