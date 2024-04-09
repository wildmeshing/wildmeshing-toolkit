#include "winding_number.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/WindingNumberOptions.hpp"
#include "internal/winding_number.hpp"

namespace wmtk::components {

void winding_number(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    WindingNumberOptions options = j.get<WindingNumberOptions>();

    // read meshes
    std::shared_ptr<Mesh> mesh = cache.read_mesh(options.input);
    std::shared_ptr<Mesh> surface_ptr = cache.read_mesh(options.filtering_base);
    TriMesh& surface_mesh = static_cast<TriMesh&>(*surface_ptr);

    // compute winding number
    auto winding_numbers = winding_number(*mesh, surface_mesh);

    wmtk::logger().info("winding number computed!");

    // get the matrix
    wmtk::utils::EigenMatrixWriter writer;
    mesh->serialize(writer);
    Eigen::MatrixXd mesh_pos;
    MatrixX<int64_t> mesh_FV;

    writer.get_position_matrix(mesh_pos);

    switch (mesh->top_simplex_type()) {
    case (PrimitiveType::Tetrahedron): {
        writer.get_TV_matrix(mesh_FV);
        break;
    }
    case (PrimitiveType::Triangle): {
        writer.get_FV_matrix(mesh_FV);
        break;
    }
    case (PrimitiveType::Edge): {
        writer.get_EV_matrix(mesh_FV);
        break;
    }
    default: throw std::runtime_error("Unsupported Mesh Type");
    }

    // construct the inside and outside mesh
    std::map<int64_t, int64_t> v_map_inside, v_map_outside;
    std::vector<Eigen::Vector3d> v_inside, v_outside;
    std::vector<std::vector<int64_t>> fv_inside, fv_outside;

    std::cout << "here" << std::endl;

    for (int64_t i = 0; i < mesh_FV.rows(); ++i) {
        if (winding_numbers[i] > options.threshold) {
            // outside
            std::vector<int64_t> row;
            for (int64_t j = 0; j < mesh_FV.cols(); ++j) {
                if (v_map_outside.find(mesh_FV(i, j)) == v_map_outside.end()) {
                    v_map_outside[mesh_FV(i, j)] = v_outside.size();
                    Eigen::Vector3d p = mesh_pos.row(mesh_FV(i, j));
                    v_outside.emplace_back(p); // map v to new vids
                }
                row.emplace_back(v_map_outside[mesh_FV(i, j)]); // update the entry of FV
            }
            fv_outside.push_back(row);
        } else {
            // inside
            std::vector<int64_t> row;
            for (int64_t j = 0; j < mesh_FV.cols(); ++j) {
                if (v_map_inside.find(mesh_FV(i, j)) == v_map_inside.end()) {
                    v_map_inside[mesh_FV(i, j)] = v_inside.size();
                    Eigen::Vector3d p = mesh_pos.row(mesh_FV(i, j));
                    v_inside.emplace_back(p); // map v to new vids
                }
                row.emplace_back(v_map_inside[mesh_FV(i, j)]); // update the entry of FV
            }
            fv_inside.push_back(row);
        }
    }

    RowVectors3d V_outside(v_outside.size(), 3), V_inside(v_inside.size(), 3);
    MatrixX<int64_t> FV_outside(fv_outside.size(), mesh_FV.cols()),
        FV_inside(fv_inside.size(), mesh_FV.cols());

    for (int64_t i = 0; i < v_inside.size(); ++i) {
        V_inside.row(i) = v_inside[i];
    }
    for (int64_t i = 0; i < v_outside.size(); ++i) {
        V_outside.row(i) = v_outside[i];
    }
    for (int64_t i = 0; i < fv_inside.size(); ++i) {
        for (int64_t j = 0; j < mesh_FV.cols(); ++j) {
            FV_inside(i, j) = fv_inside[i][j];
        }
    }
    for (int64_t i = 0; i < fv_outside.size(); ++i) {
        for (int64_t j = 0; j < mesh_FV.cols(); ++j) {
            FV_outside(i, j) = fv_outside[i][j];
        }
    }

    wmtk::logger().info("inside and outside mesh constructed");

    switch (mesh->top_simplex_type()) {
    case (PrimitiveType::Tetrahedron): {
        std::shared_ptr<TetMesh> m_out = std::make_shared<wmtk::TetMesh>();
        std::shared_ptr<TetMesh> m_in = std::make_shared<wmtk::TetMesh>();
        m_out->initialize(FV_outside);
        m_in->initialize(FV_inside);
        mesh_utils::set_matrix_attribute(V_outside, "vertices", PrimitiveType::Vertex, *m_out);
        mesh_utils::set_matrix_attribute(V_inside, "vertices", PrimitiveType::Vertex, *m_in);
        cache.write_mesh(*m_in, options.output + "_inside");
        cache.write_mesh(*m_out, options.output + "_outside");
        break;
    }
    case (PrimitiveType::Triangle): {
        std::shared_ptr<TriMesh> m_out = std::make_shared<wmtk::TriMesh>();
        std::shared_ptr<TriMesh> m_in = std::make_shared<wmtk::TriMesh>();
        m_out->initialize(FV_outside);
        m_in->initialize(FV_inside);
        mesh_utils::set_matrix_attribute(V_outside, "vertices", PrimitiveType::Vertex, *m_out);
        mesh_utils::set_matrix_attribute(V_inside, "vertices", PrimitiveType::Vertex, *m_in);
        cache.write_mesh(*m_in, options.output + "_inside");
        cache.write_mesh(*m_out, options.output + "_outside");
        break;
    }
    case (PrimitiveType::Edge): {
        std::shared_ptr<EdgeMesh> m_out = std::make_shared<wmtk::EdgeMesh>();
        std::shared_ptr<EdgeMesh> m_in = std::make_shared<wmtk::EdgeMesh>();
        m_out->initialize(FV_outside);
        m_in->initialize(FV_inside);
        mesh_utils::set_matrix_attribute(V_outside, "vertices", PrimitiveType::Vertex, *m_out);
        mesh_utils::set_matrix_attribute(V_inside, "vertices", PrimitiveType::Vertex, *m_in);
        cache.write_mesh(*m_in, options.output + "_inside");
        cache.write_mesh(*m_out, options.output + "_outside");
        break;
    }
    default: throw std::runtime_error("Unsupported Mesh Type");
    }

    wmtk::logger().info("winding number filtering finished");
}

} // namespace wmtk::components