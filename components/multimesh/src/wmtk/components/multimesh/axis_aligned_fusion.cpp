#include "axis_aligned_fusion.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>


namespace wmtk::components::multimesh {

std::shared_ptr<Mesh>
axis_aligned_fusion(const Mesh& mesh, const std::vector<bool>& operating_axis, double eps)
{
    // get mesh dimension and checks
    int64_t mesh_dim = mesh.top_cell_dimension();


    std::map<std::string, std::vector<int64_t>> names;

    // TODO: a lot of matrix copies to remove

    switch (mesh_dim) {
    case (2): {
        if (operating_axis[2]) {
            wmtk::logger().warn("Fusion on Z axis is not supported for 2D mesh.");
        }

        MatrixX<double> V;
        MatrixX<int64_t> FV;
        wmtk::utils::EigenMatrixWriter writer;
        mesh.serialize(writer);

        writer.get_position_matrix(V);
        writer.get_FV_matrix(FV);

        assert(V.rows() == mesh.get_all(PrimitiveType::Vertex).size());
        assert(FV.rows() == mesh.get_all(PrimitiveType::Triangle).size());

        // rescale to [0, 1]
        Eigen::MatrixXd V_rescale(V.rows(), V.cols());

        Eigen::VectorXd min_pos = V.colwise().minCoeff();

        // if (abs(min_pos[2] - 0.0) > eps)
        //     throw std::runtime_error("has non-zero z coordinate on 2d mesh");

        for (int64_t i = 0; i < V_rescale.rows(); ++i) {
            V_rescale.row(i) = V.row(i) - min_pos.transpose();
        }

        Eigen::VectorXd max_pos = V_rescale.colwise().maxCoeff();
        for (int64_t i = 0; i < V_rescale.rows(); ++i) {
            V_rescale(i, 0) /= max_pos[0];
            V_rescale(i, 1) /= max_pos[1];
        }


        // debug code
        // std::cout << FV << std::endl;
        // std::cout << V << std::endl;

        std::map<int64_t, int64_t> vertex_map;

        std::array<std::vector<std::pair<int64_t, Eigen::VectorXd>>, 2> vertices_on_zero;
        std::array<std::vector<std::pair<int64_t, Eigen::VectorXd>>, 2> vertices_on_one;

        int64_t v00 = -1;
        int64_t v01 = -1;
        int64_t v10 = -1;
        int64_t v11 = -1;

        for (int64_t i = 0; i < V_rescale.rows(); ++i) {
            const auto& pos = V_rescale.row(i);

            if (abs(pos[0] - 0.0) < eps) {
                vertices_on_zero[0].emplace_back(std::make_pair(i, pos));
            }
            if (abs(pos[1] - 0.0) < eps) {
                vertices_on_zero[1].emplace_back(std::make_pair(i, pos));
            }
            if (abs(pos[0] - 1.0) < eps) {
                vertices_on_one[0].emplace_back(std::make_pair(i, pos));
            }
            if (abs(pos[1] - 1.0) < eps) {
                vertices_on_one[1].emplace_back(std::make_pair(i, pos));
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
            if (!operating_axis[axis]) continue;

            if (vertices_on_zero[axis].size() != vertices_on_one[axis].size()) {
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
        if (operating_axis[0] && operating_axis[1]) {
            vertex_map[v00] = v00;
            vertex_map[v01] = v00;
            vertex_map[v10] = v00;
            vertex_map[v11] = v00;
        }

        // create periodic mesh
        RowVectors3l FV_new(FV.rows(), 3);

        for (int64_t i = 0; i < FV.rows(); ++i) {
            for (int64_t k = 0; k < 3; ++k) {
                if (vertex_map.find(FV(i, k)) != vertex_map.end()) {
                    FV_new(i, k) = vertex_map[FV(i, k)];
                } else {
                    FV_new(i, k) = FV(i, k);
                }
            }
        }

        Eigen::MatrixXd V_new_tmp(V.rows(), V.cols());

        // remove unused vertices
        std::map<int64_t, int64_t> v_consolidate_map;

        int64_t v_valid_cnt = 0;
        for (int64_t i = 0; i < V.rows(); ++i) {
            if (vertex_map.find(i) == vertex_map.end() || vertex_map[i] == i) {
                for (int k = 0; k < V_rescale.cols(); ++k) {
                    V_new_tmp(v_valid_cnt, k) = V(i, k);
                }
                v_consolidate_map[i] = v_valid_cnt++;

                // std::cout << V_new_tmp << std::endl << std::endl;
            }
        }

        Eigen::MatrixXd V_new(v_valid_cnt, V.cols());
        V_new = V_new_tmp.block(0, 0, v_valid_cnt, V.cols());

        // std::cout << V_new << std::endl << std::endl;


        // update FV_new
        for (int64_t i = 0; i < FV_new.rows(); ++i) {
            for (int64_t k = 0; k < FV_new.cols(); ++k) {
                FV_new(i, k) = v_consolidate_map[FV_new(i, k)];
            }
        }


        TriMesh fusion_mesh;
        fusion_mesh.initialize(FV_new);
        // mesh_utils::set_matrix_attribute(V_new, "vertices", PrimitiveType::Vertex, fusion_mesh);

        // TriMesh& m = dynamic_cast<TriMesh&>(*mesh);

        // make a copy
        // TriMesh child_mesh(std::move(m));
        // std::shared_ptr<TriMesh> child_ptr = std::make_shared<TriMesh>(std::move(m));

        TriMesh position_mesh;
        position_mesh.initialize(FV);
        mesh_utils::set_matrix_attribute(
            V_rescale,
            "vertices",
            PrimitiveType::Vertex,
            position_mesh);

        // std::shared_ptr<TriMesh> child_ptr = std::make_shared<TriMesh>(std::move(position_mesh));

        // auto child_map = multimesh::same_simplex_dimension_bijection(fusion_mesh, *child_ptr);


        // fusion_mesh.register_child_mesh(child_ptr, child_map);

        // names["periodic"] = fusion_mesh.absolute_multi_mesh_id();
        // names["position"] = child_ptr->absolute_multi_mesh_id();

        // cache.write_mesh(fusion_mesh, options.name, names);

        // break;

        std::shared_ptr<TriMesh> child_ptr = std::make_shared<TriMesh>(std::move(position_mesh));
        std::shared_ptr<TriMesh> parent_ptr = std::make_shared<TriMesh>(std::move(fusion_mesh));

        auto child_map = wmtk::multimesh::same_simplex_dimension_bijection(*parent_ptr, *child_ptr);
        parent_ptr->register_child_mesh(child_ptr, child_map);

        return parent_ptr;
    }
    case (3): {
        MatrixX<double> V;
        MatrixX<int64_t> TV;

        wmtk::utils::EigenMatrixWriter writer;
        mesh.serialize(writer);

        writer.get_position_matrix(V);
        writer.get_TV_matrix(TV);

        assert(V.rows() == mesh.get_all(PrimitiveType::Vertex).size());
        assert(TV.rows() == mesh.get_all(PrimitiveType::Tetrahedron).size());

        // rescale to [0, 1]
        RowVectors3d V_rescale(V.rows(), 3);

        Vector3d min_pos = V.colwise().minCoeff();

        for (int64_t i = 0; i < V_rescale.rows(); ++i) {
            V_rescale.row(i) = V.row(i) - min_pos.transpose();
        }

        Vector3d max_pos = V_rescale.colwise().maxCoeff();
        for (int64_t i = 0; i < V_rescale.rows(); ++i) {
            V_rescale(i, 0) /= max_pos[0];
            V_rescale(i, 1) /= max_pos[1];
            V_rescale(i, 2) /= max_pos[2];
        }

        std::map<int64_t, int64_t> vertex_map;

        std::array<std::vector<std::pair<int64_t, Eigen::VectorXd>>, 3> vertices_on_zero;
        std::array<std::vector<std::pair<int64_t, Eigen::VectorXd>>, 3> vertices_on_one;

        for (int64_t i = 0; i < V_rescale.rows(); ++i) {
            const auto& pos = V_rescale.row(i);

            for (int k = 0; k < 3; ++k) {
                if (abs(pos[k] - 0.0) < eps) {
                    vertices_on_zero[k].push_back(std::make_pair(i, pos));
                }
                if (abs(pos[k] - 1.0) < eps) {
                    vertices_on_one[k].push_back(std::make_pair(i, pos));
                }
            }
        }

        // merge vertices
        for (int axis = 0; axis < 3; ++axis) {
            if (!operating_axis[axis]) continue;

            if (vertices_on_zero[axis].size() != vertices_on_one[axis].size()) {
                throw std::runtime_error("vertices size on the fusion axis does not match!");
            }

            auto cmp = [&](const std::pair<int64_t, Eigen::VectorXd>& a,
                           const std::pair<int64_t, Eigen::VectorXd>& b) {
                if (abs(a.second[(axis + 2) % 3] - b.second[(axis + 2) % 3]) < eps) {
                    return a.second[(axis + 1) % 3] < b.second[(axis + 1) % 3];
                } else {
                    return a.second[(axis + 2) % 3] < b.second[(axis + 2) % 3];
                }
            };

            std::sort(vertices_on_zero[axis].begin(), vertices_on_zero[axis].end(), cmp);
            std::sort(vertices_on_one[axis].begin(), vertices_on_one[axis].end(), cmp);

            for (int64_t i = 0; i < vertices_on_zero[axis].size(); ++i) {
                assert(
                    abs(vertices_on_zero[axis][i].second[(axis + 1) % 3] -
                        vertices_on_one[axis][i].second[(axis + 1) % 3]) < eps &&
                    abs(vertices_on_zero[axis][i].second[(axis + 2) % 3] -
                        vertices_on_one[axis][i].second[(axis + 2) % 3]) < eps);

                vertex_map[vertices_on_one[axis][i].first] = vertices_on_zero[axis][i].first;
            }
        }

        // // create periodic mesh
        RowVectors4l TV_new(TV.rows(), 4);

        for (int64_t i = 0; i < TV.rows(); ++i) {
            for (int64_t k = 0; k < 4; ++k) {
                if (vertex_map.find(TV(i, k)) != vertex_map.end()) {
                    int64_t v_root = vertex_map[TV(i, k)];
                    while (vertex_map.find(v_root) != vertex_map.end() &&
                           vertex_map[v_root] != v_root) {
                        v_root = vertex_map[v_root];
                    }
                    TV_new(i, k) = v_root;
                } else {
                    TV_new(i, k) = TV(i, k);
                }
            }
        }

        // std::cout << TV_new << std::endl << std::endl;


        RowVectors3d V_new_tmp(V.rows(), 3);

        // remove unused vertices
        std::map<int64_t, int64_t> v_consolidate_map;

        int64_t v_valid_cnt = 0;
        for (int64_t i = 0; i < V.rows(); ++i) {
            if (vertex_map.find(i) == vertex_map.end() || vertex_map[i] == i) {
                V_new_tmp(v_valid_cnt, 0) = V(i, 0);
                V_new_tmp(v_valid_cnt, 1) = V(i, 1);
                V_new_tmp(v_valid_cnt, 2) = V(i, 2);
                v_consolidate_map[i] = v_valid_cnt++;

                // std::cout << V_new_tmp << std::endl << std::endl;
            }
        }

        RowVectors3d V_new(v_valid_cnt, 3);
        V_new = V_new_tmp.block(0, 0, v_valid_cnt, 3);

        // std::cout << V_new << std::endl << std::endl;


        // update TV_new
        for (int64_t i = 0; i < TV_new.rows(); ++i) {
            for (int64_t k = 0; k < TV_new.cols(); ++k) {
                TV_new(i, k) = v_consolidate_map[TV_new(i, k)];
            }
        }

        // std::cout << TV_new << std::endl << std::endl;


        TetMesh fusion_mesh;
        fusion_mesh.initialize(TV_new);
        // mesh_utils::set_matrix_attribute(V_new, "vertices", PrimitiveType::Vertex, fusion_mesh);

        TetMesh position_mesh;
        position_mesh.initialize(TV);
        mesh_utils::set_matrix_attribute(
            V_rescale,
            "vertices",
            PrimitiveType::Vertex,
            position_mesh);

        std::shared_ptr<TetMesh> child_ptr = std::make_shared<TetMesh>(std::move(position_mesh));
        std::shared_ptr<TetMesh> parent_ptr = std::make_shared<TetMesh>(std::move(fusion_mesh));

        auto child_map = wmtk::multimesh::same_simplex_dimension_bijection(*parent_ptr, *child_ptr);
        parent_ptr->register_child_mesh(child_ptr, child_map);

        return parent_ptr;
    }
    default: {
        throw std::runtime_error("mesh dimension not supported");
    }
    }
    return {};
}
} // namespace wmtk::components::multimesh
