#include "fusion.hpp"
#include "FusionOptions.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>


namespace wmtk::components {

void fusion(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    // load mesh
    FusionOptions options = j.get<FusionOptions>();
    std::shared_ptr<Mesh> mesh = cache.read_mesh(options.input);

    // get fusion axis
    int64_t fusion_axis = static_cast<int64_t>(options.fusion_axis);
    // int64_t fusion_axis = options.fusion_axis;


    // get mesh dimension and checks
    int64_t mesh_dim = mesh->top_cell_dimension();

    double eps = 1e-10;

    switch (mesh_dim) {
    case (2): {
        if (fusion_axis == 2) {
            throw std::runtime_error("cannot fusion axis Z of a 2D mesh");
        }

        MatrixX<double> V;
        MatrixX<int64_t> FV;
        wmtk::utils::EigenMatrixWriter writer;
        mesh->serialize(writer);

        writer.get_position_matrix(V);
        writer.get_FV_matrix(FV);

        assert(V.rows() == mesh->get_all(PrimitiveType::Vertex).size());
        assert(FV.rows() == mesh->get_all(PrimitiveType::Face).size());

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

        for (int64_t i = 0; i < V.rows(); ++i) {
            const auto& pos = V.row(i);

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
        if (fusion_axis == 3) {
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
        // V_new.resize(v_valid_cnt, 3);
        V_new = V_new_tmp.block(0, 0, v_valid_cnt, 3);

        // std::cout << V_new << std::endl << std::endl;


        // update FV_new
        for (int64_t i = 0; i < FV_new.rows(); ++i) {
            for (int64_t k = 0; k < FV_new.cols(); ++k) {
                FV_new(i, k) = v_consolidate_map[FV_new(i, k)];
            }
        }


        TriMesh fusion_mesh;
        fusion_mesh.initialize(FV_new);
        mesh_utils::set_matrix_attribute(V_new, "vertices", PrimitiveType::Vertex, fusion_mesh);

        cache.write_mesh(fusion_mesh, options.name);

        break;
    }
    case (3): {
        MatrixX<double> V;
        MatrixX<int64_t> TV;

        wmtk::utils::EigenMatrixWriter writer;
        mesh->serialize(writer);

        writer.get_position_matrix(V);
        writer.get_TV_matrix(TV);

        assert(V.rows() == mesh->get_all(PrimitiveType::Vertex).size());
        assert(TV.rows() == mesh->get_all(PrimitiveType::Tetrahedron).size());

        std::map<int64_t, int64_t> vertex_map;

        std::array<std::vector<std::pair<int64_t, Eigen::VectorXd>>, 3> vertices_on_zero;
        std::array<std::vector<std::pair<int64_t, Eigen::VectorXd>>, 3> vertices_on_one;

        std::array<int64_t, 8> v_corner = {{-1, -1, -1, -1, -1, -1, -1, -1}};
        std::array<std::array<double, 3>, 8> v_corner_coord = {
            {{{0.0, 0.0, 0.0}},
             {{0.0, 0.0, 1.0}},
             {{0.0, 1.0, 0.0}},
             {{0.0, 1.0, 1.0}},
             {{1.0, 0.0, 0.0}},
             {{1.0, 0.0, 1.0}},
             {{1.0, 1.0, 0.0}},
             {{1.0, 1.0, 1.0}}}};

        for (int64_t i = 0; i < V.rows(); ++i) {
            const auto& pos = V.row(i);

            for (int k = 0; k < 3; ++k) {
                if (abs(pos[k] - 0.0) < eps) {
                    vertices_on_zero[k].push_back(std::make_pair(i, pos));
                }
                if (abs(pos[k] - 1.0) < eps) {
                    vertices_on_one[k].push_back(std::make_pair(i, pos));
                }
            }

            // corners
            for (int k = 0; k < 8; ++k) {
                if (abs(pos[0] - v_corner_coord[k][0]) < eps &&
                    abs(pos[1] - v_corner_coord[k][1]) < eps &&
                    abs(pos[2] - v_corner_coord[k][2])) {
                    if (v_corner[k] != -1)
                        throw std::runtime_error(
                            "More than 1 vertices on corner " + std::to_string(k) + ".");
                    v_corner[k] = i;
                }
            }
        }


        // TODO: use union find to merge vertices
        // merge vertices
        // for (int axis = 0; axis < 3; ++axis) {
        //     if (axis != fusion_axis && fusion_axis != 3) continue;

        //     if (vertices_on_zero[axis].size() == vertices_on_one[axis].size()) {
        //         throw std::runtime_error("vertices size on the fusion axis does not match!");
        //     }

        //     auto cmp = [&](const std::pair<int64_t, Eigen::VectorXd>& a,
        //                    const std::pair<int64_t, Eigen::VectorXd>& b) {
        //         if (abs(a.second[(axis + 2) % 3] - b.second[(axis + 2) % 3]) < eps) {
        //             return a.second[(axis + 1) % 3] < b.second[(axis + 1) % 3];
        //         } else {
        //             return a.second[(axis + 2) % 3] < b.second[(axis + 2) % 3];
        //         }
        //     };

        //     std::sort(vertices_on_zero[axis].begin(), vertices_on_zero[axis].end(), cmp);
        //     std::sort(vertices_on_one[axis].begin(), vertices_on_one[axis].end(), cmp);

        //     for (int64_t i = 0; i < vertices_on_zero[axis].size(); ++i) {
        //         assert(
        //             abs(vertices_on_zero[axis][i].second[(axis + 1) % 3] -
        //                 vertices_on_one[axis][i].second[(axis + 1) % 3]) < eps &&
        //             abs(vertices_on_zero[axis][i].second[(axis + 2) % 3] -
        //                 vertices_on_one[axis][i].second[(axis + 2) % 3]) < eps);

        //         vertex_map[vertices_on_one[axis][i].first] = vertices_on_zero[axis][i].first;
        //     }

        //     // special case for fusion all axis
        //     // TODO: check if this is correct for hypertorus
        //     if (fusion_axis == 3) {
        //         vertex_map[v_corner[0]] = v_corner[0];
        //         vertex_map[v_corner[1]] = v_corner[0];
        //         vertex_map[v_corner[2]] = v_corner[0];
        //         vertex_map[v_corner[3]] = v_corner[0];
        //         vertex_map[v_corner[4]] = v_corner[0];
        //         vertex_map[v_corner[5]] = v_corner[0];
        //         vertex_map[v_corner[6]] = v_corner[0];
        //         vertex_map[v_corner[7]] = v_corner[0];
        //     }
        // }

        // // create periodic mesh
        // RowVectors4l TV_new(TV.rows(), 4);

        // for (int64_t i = 0; i < TV.rows(); ++i) {
        //     for (int64_t k = 0; k < 4; ++k) {
        //         if (vertex_map.find(TV(i, k)) != vertex_map.end()) {
        //             TV_new(i, k) = vertex_map[TV(i, k)];
        //         } else {
        //             TV_new(i, k) = TV(i, k);
        //         }
        //     }
        // }

        // RowVectors3d V_new(V.rows(), 3);

        // std::map<int64_t, int64_t> v_consolidate_map;

        // int64_t v_valid_cnt = 0;
        // for (int64_t i = 0; i < V.rows(); ++i) {
        //     // not in map or map to itself
        //     if (vertex_map.find(i) == vertex_map.end() || vertex_map[i] == i) {
        //         V_new(v_valid_cnt, 0) = V(i, 0);
        //         V_new(v_valid_cnt, 1) = V(i, 1);
        //         V_new(v_valid_cnt, 2) = V(i, 2);
        //         v_consolidate_map[i] = v_valid_cnt++;
        //     }
        // }

        // V_new.resize(v_valid_cnt, 3);

        break;
    }
    default: {
        throw std::runtime_error("mesh dimension not supported");
    }
    }
}
} // namespace wmtk::components