#include "c1_simplification.hpp"

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include "c1_multimesh.hpp"
#include "c1_utils.hpp"

#include <jse/jse.h>

#include <MshIO/mshio.h>
#include <memory>
#include <vector>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>

#include <igl/readMSH.h>
#include <igl/readOBJ.h>
#include <spdlog/common.h>

#include "c1_simplification_spec.hpp"

namespace wmtk::components::c1_simplification {

void c1_simplification(nlohmann::json json_params)
{
    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, c1_simplification_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, c1_simplification_spec);
    }

    std::string surface_mesh_file = json_params["surface_mesh"];
    std::string tet_mesh_file = json_params["tet_mesh"];
    std::string uv_mesh_file = json_params["uv_mesh"];
    std::string dofs_file = json_params["dofs"];
    std::string cones_file = json_params["cones"];
    std::string tracked_vertices_filename = json_params["tracked_vertices_file"];
    std::string s2t_v_map_file = json_params["s2t_v_map"];

    std::string output_file = json_params["output"];

    double threshold = json_params["threshold"];
    int logger_level = json_params["log_level"];

    // TODO: make this a json option
    switch (logger_level) {
    case 0: wmtk::logger().set_level(spdlog::level::trace); break;
    case 1: wmtk::logger().set_level(spdlog::level::debug); break;
    case 2: wmtk::logger().set_level(spdlog::level::info); break;

    default: wmtk::logger().set_level(spdlog::level::info);
    }
    // wmtk::logger().set_level(spdlog::level::debug); // 0 trace, 1 debug, 2 info

    wmtk::logger().set_level(spdlog::level::info);
    // read surface
    Eigen::MatrixXd s_V, _uv, _N;
    Eigen::MatrixXi s_F, _FT, _FN;
    igl::readOBJ(surface_mesh_file, s_V, _uv, _N, s_F, _FT, _FN);

    // get bbox
    Eigen::Vector3d bbox_max, bbox_min;
    bbox_max = s_V.colwise().maxCoeff();
    bbox_min = s_V.colwise().minCoeff();
    double diag_length = (bbox_max - bbox_min).norm();

    // read uv and initialize
    Eigen::MatrixXd _V, uv_V, N;
    Eigen::MatrixXi _F, uv_F, FN;
    igl::readOBJ(uv_mesh_file, _V, uv_V, N, _F, uv_F, FN);

    std::shared_ptr<MMUVMesh> uv_mesh_ptr = std::make_shared<MMUVMesh>();

    uv_mesh_ptr->init_from_eigen(uv_V, uv_F);
    uv_mesh_ptr->output_uv_mesh(output_file + "_uv_initial.obj");

    // read tetmesh and initialize
    Eigen::MatrixXd t_V;
    Eigen::MatrixXi t_T;
    std::shared_ptr<MMTetMesh> tet_mesh_ptr = nullptr;
    if (tet_mesh_file != "") {
        read_tetmesh(tet_mesh_file, t_V, t_T);
        tet_mesh_ptr = std::make_shared<MMTetMesh>();
        tet_mesh_ptr->init_from_eigen(t_V, t_T);
    }

    // read dofs
    std::ifstream dofs(dofs_file);
    std::vector<Eigen::Matrix<double, 12, 3>> face_dofs(s_F.rows());

    for (size_t i = 0; i < s_F.rows(); ++i) {
        for (int row = 0; row < 12; ++row) {
            for (int col = 0; col < 3; ++col) {
                dofs >> face_dofs[i](row, col);
            }
        }
    }
    dofs.close();

    // read cones
    std::ifstream cones(cones_file);
    std::vector<size_t> cone_vids;

    size_t cid;
    while (cones >> cid) {
        cone_vids.push_back(cid);
    }
    cones.close();

    // read tracked vertices
    std::ifstream tracked_vertices_file(tracked_vertices_filename);
    std::vector<MMSurfaceMesh::tracked_vertex> tracked_vertices;


    // file << std::setprecision(16) << info.fid << " " << info.pos_3d[0] << " "
    //  << info.pos_3d[1] << " " << info.pos_3d[2] << " " << info.pos_uv[0]
    //  << " " << info.pos_uv[1] << " " << info.dfdu[0] << " " << info.dfdu[1]
    //  << " " << info.dfdu[2] << " " << info.dfdv[0] << " " << info.dfdv[1]
    //  << " " << info.dfdv[2] << " " << info.micro_id << " " << info.micro_u
    //  << " " << info.micro_v << " " << info.micro_v0[0] << " "
    //  << info.micro_v0[1] << " " << info.micro_v1[0] << " "
    //  << info.micro_v1[1] << " " << info.micro_v2[0] << " "
    //  << info.micro_v2[1] << " " << info.area << std::endl;


    int64_t fid, micro_id;
    double px, py, pz, s, t, dux, duy, duz, dvx, dvy, dvz, micro_u, micro_v, mv0_u, mv0_v, mv1_u,
        mv1_v, mv2_u, mv2_v, area;
    int64_t tracked_v_cnt = 0;
    while (tracked_vertices_file >> fid >> px >> py >> pz >> s >> t >> dux >> duy >> duz >> dvx >>
           dvy >> dvz >> micro_id >> micro_u >> micro_v >> mv0_u >> mv0_v >> mv1_u >> mv1_v >>
           mv2_u >> mv2_v >> area) {
        MMSurfaceMesh::tracked_vertex tracked_v;
        tracked_v.uv_pos = Vector2d(s, t);
        tracked_v.surface_pos = Vector3d(px, py, pz);
        tracked_v.in_tri_id = fid;
        tracked_v.vid = tracked_v_cnt;

        tracked_v.dfdu = Vector3d(dux, duy, duz);
        tracked_v.dfdv = Vector3d(dvx, dvy, dvz);
        tracked_v.micro_id = micro_id;
        tracked_v.micro_u = micro_u;
        tracked_v.micro_v = micro_v;
        tracked_v.micro_v0 = Vector2d(mv0_u, mv0_v);
        tracked_v.micro_v1 = Vector2d(mv1_u, mv1_v);
        tracked_v.micro_v2 = Vector2d(mv2_u, mv2_v);
        tracked_v.area = area;

        tracked_vertices.push_back(tracked_v);

        tracked_v_cnt++;
    }

    tracked_vertices_file.close();


    // read vertex mapping
    std::map<int64_t, int64_t> s2t_vid_map;
    int64_t v_cnt = 0;
    if (tet_mesh_ptr != nullptr) {
        std::ifstream s2t_vid_map_file(s2t_v_map_file);

        int64_t t_vid;
        while (s2t_vid_map_file >> t_vid) {
            s2t_vid_map[v_cnt] = t_vid;
            v_cnt++;
        }
        s2t_vid_map_file.close();
    }


    wmtk::logger().info("meshes and dofs read.");

    MMSurfaceMesh surface_mesh(tet_mesh_ptr, uv_mesh_ptr, threshold * diag_length);

    surface_mesh.init_from_eigen_with_map_and_dofs(
        s_V,
        s_F,
        s2t_vid_map,
        face_dofs,
        cone_vids,
        tracked_vertices);

    wmtk::logger().info("initialized multimesh. tetmesh involved: {}", tet_mesh_ptr != nullptr);
    surface_mesh.output_tracked_vertices("initial_tracked_vertices.obj");

    int suc_cnt = 0;
    int iter_cnt = 0;
    int try_cnt = 0;
    int total_try_cnt = 0;
    int total_suc_cnt = 0;

    surface_mesh.multimesh_consolidated_output(
        output_file + "_multi_" + std::to_string(total_suc_cnt));
    surface_mesh.output_tracked_vertices(
        output_file + "_tracked_vertices_" + std::to_string(total_suc_cnt) + ".obj");
    surface_mesh.write_tracked_vertices_info(
        output_file + "_tracked_vertices_info_" + std::to_string(total_suc_cnt) + ".txt");


    auto edge_length_compare = [&](const MMSurfaceMesh::Tuple& e1, const MMSurfaceMesh::Tuple& e2) {
        auto uv_e1 = surface_mesh.map_to_uv_edge_tuples(e1)[0];
        auto uv_e2 = surface_mesh.map_to_uv_edge_tuples(e2)[0];
        double e1_len = (surface_mesh.uvmesh_ptr->v_attrs[uv_e1.vid(*surface_mesh.uvmesh_ptr)].pos -
                         surface_mesh.uvmesh_ptr
                             ->v_attrs[uv_e1.switch_vertex(*surface_mesh.uvmesh_ptr)
                                           .vid(*surface_mesh.uvmesh_ptr)]
                             .pos)
                            .norm();
        double e2_len = (surface_mesh.uvmesh_ptr->v_attrs[uv_e2.vid(*surface_mesh.uvmesh_ptr)].pos -
                         surface_mesh.uvmesh_ptr
                             ->v_attrs[uv_e2.switch_vertex(*surface_mesh.uvmesh_ptr)
                                           .vid(*surface_mesh.uvmesh_ptr)]
                             .pos)
                            .norm();
        return e1_len < e2_len;
    };


    for (int iter = 0; iter < 5; ++iter) {
        wmtk::logger().info("----------------------- pass {} ----------------------", iter);
        wmtk::logger().info("executing collapse ...");

        suc_cnt = 0;
        iter_cnt = 0;
        try_cnt = 0;
        total_try_cnt = 0;
        total_suc_cnt = 0;
        // collpase
        do {
            // TODO: test use, remove
            // break;

            suc_cnt = 0;
            try_cnt = 0;
            auto edge_tuples = surface_mesh.get_edges();
            std::sort(edge_tuples.begin(), edge_tuples.end(), edge_length_compare);
            wmtk::logger().info("done sorting edge length");

            for (auto& e : edge_tuples) {
                if (e.is_valid(surface_mesh)) {
                    // check for seams
                    auto uv_es = surface_mesh.map_to_uv_edge_tuples(e);
                    // if (uv_es.size() < 2) {
                    //     continue;
                    // }

                    wmtk::logger().debug(
                        "try {} collapse e {} {}",
                        try_cnt,
                        e.vid(surface_mesh),
                        e.switch_vertex(surface_mesh).vid(surface_mesh));

                    try_cnt++;
                    if (surface_mesh.multimesh_collapse_edge(e)) {
                        wmtk::logger().debug("succeeded");

                        suc_cnt++;
                        total_suc_cnt++;

                        // uv_mesh_ptr->output_uv_mesh("uv_" + std::to_string(total_suc_cnt) +
                        // ".obj"); surface_mesh.output_tracked_vertices(
                        //     "tracked_vertices" + std::to_string(total_suc_cnt) + ".obj");
                        // surface_mesh.multimesh_consolidated_output(
                        //     output_file + "_multi_" + std::to_string(total_suc_cnt));
                        // surface_mesh.write_tracked_vertices_info(
                        //     output_file + "_tracked_vertices_info_" +
                        //     std::to_string(total_suc_cnt) +
                        //     ".txt");
                        // surface_mesh.output_tracked_vertices(
                        //     "out_tracked_vertices_" + std::to_string(total_suc_cnt) + ".obj");

                        // break;
                    } else if (surface_mesh.multimesh_collapse_edge(
                                   e.switch_vertex(surface_mesh))) {
                        wmtk::logger().debug("succeeded other direction");

                        suc_cnt++;
                        total_suc_cnt++;

                        // surface_mesh.multimesh_consolidated_output(
                        //     output_file + "_multi_" + std::to_string(total_suc_cnt));
                        // surface_mesh.write_tracked_vertices_info(
                        //     output_file + "_tracked_vertices_info_" +
                        //     std::to_string(total_suc_cnt) +
                        //     ".txt");
                        // surface_mesh.output_tracked_vertices(
                        //     "out_tracked_vertices_" + std::to_string(total_suc_cnt) + ".obj");


                    } else {
                        wmtk::logger().debug("failed");
                    }

                    // uv_mesh_ptr->output_uv_mesh(std::to_string(try_cnt) + "_uv.obj", false);
                    // try_cnt++;

                    // TODO: debug use, remove
                    // if (try_cnt > 4) break;
                }
                // if (total_suc_cnt > 1) break;
                // break;
            }


            wmtk::logger().info(
                "iter {} collapse success cnt: {} out of {} tries",
                iter_cnt,
                suc_cnt,
                try_cnt);
            iter_cnt++;

            total_try_cnt += try_cnt;

            // if (total_suc_cnt > 0) break;
            // TODO: remove
            // if (iter_cnt > 6) break;
            if (suc_cnt < 5) break;
        } while (suc_cnt > 0);

        wmtk::logger().info(
            "total collapse success cnt: {} out of {} tries",
            total_suc_cnt,
            total_try_cnt);

        // surface_mesh.multimesh_consolidated_output(output_file + "_multi_after_collapse");
        // surface_mesh.output_tracked_vertices(output_file +
        // "_tracked_vertices_after_collapse.obj"); surface_mesh.write_tracked_vertices_info(
        //     output_file + "_tracked_vertices_info_after_collapse.txt");

        // swap
        suc_cnt = 0;
        iter_cnt = 0;
        try_cnt = 0;
        total_try_cnt = 0;
        total_suc_cnt = 0;

        // test check
        // for (const auto& e : surface_mesh.uvmesh_ptr->get_edges()) {
        //     auto ee = e.switch_face(*surface_mesh.uvmesh_ptr).has_value();
        // }

        wmtk::logger().info("executing swap ...");

        do {
            // TODO: test use, remove
            // break;

            suc_cnt = 0;
            try_cnt = 0;
            auto edge_tuples = surface_mesh.get_edges();
            for (auto& e : edge_tuples) {
                if (e.is_valid(surface_mesh)) {
                    // check for seams
                    auto uv_es = surface_mesh.map_to_uv_edge_tuples(e);
                    // if (uv_es.size() < 2) {
                    //     continue;
                    // }

                    wmtk::logger().debug(
                        "try {} swap e {} {}",
                        try_cnt,
                        e.vid(surface_mesh),
                        e.switch_vertex(surface_mesh).vid(surface_mesh));

                    // if (total_suc_cnt == 623) {
                    //     std::cout << 623 << std::endl;
                    // }

                    try_cnt++;

                    // if (total_suc_cnt == 142 && try_cnt == 1258) {
                    //     std::cout << "try 1258" << std::endl;
                    // }
                    if (surface_mesh.multimesh_swap_edge(e)) {
                        wmtk::logger().debug("succeeded");

                        suc_cnt++;
                        total_suc_cnt++;

                        // // test check
                        // for (const auto& eee : surface_mesh.uvmesh_ptr->get_edges()) {
                        //     auto ee = eee.switch_face(*surface_mesh.uvmesh_ptr).has_value();
                        // }


                        // uv_mesh_ptr->output_uv_mesh("uv_" + std::to_string(total_suc_cnt) +
                        // ".obj"); surface_mesh.output_tracked_vertices(
                        //     "tracked_vertices" + std::to_string(total_suc_cnt) + ".obj");
                        // surface_mesh.multimesh_consolidated_output(
                        //     output_file + "_multi_" + std::to_string(total_suc_cnt));
                        // surface_mesh.write_tracked_vertices_info(
                        //     output_file + "_tracked_vertices_info_" +
                        //     std::to_string(total_suc_cnt) +
                        //     ".txt");
                        // surface_mesh.output_tracked_vertices(
                        //     "out_tracked_vertices_" + std::to_string(total_suc_cnt) + ".obj");

                        // break;
                    } else {
                        wmtk::logger().debug("failed");
                    }

                    // uv_mesh_ptr->output_uv_mesh(std::to_string(try_cnt) + "_uv.obj", false);
                    // try_cnt++;

                    // TODO: debug use, remove
                    // if (try_cnt > 4) break;
                }
                // if (total_suc_cnt > 1) break;
                // break;
            }


            wmtk::logger()
                .info("iter {} swap success cnt: {} out of {} tries", iter_cnt, suc_cnt, try_cnt);
            iter_cnt++;

            total_try_cnt += try_cnt;

            // if (total_suc_cnt > 0) break;
        } while (suc_cnt > 0);

        wmtk::logger().info(
            "total swap success cnt: {} out of {} tries",
            total_suc_cnt,
            total_try_cnt);
    }

    surface_mesh.multimesh_consolidated_output(output_file + "_multi");
    surface_mesh.output_tracked_vertices(output_file + "_tracked_vertices.obj");
    surface_mesh.write_tracked_vertices_info(output_file + "_tracked_vertices_info.txt");

    surface_mesh.multimesh_consolidated_micro_tri_output(output_file + "_micro_uv");
    surface_mesh.write_micro_triangle_tracked_vertices_info(
        output_file + "_tracked_vertices_info_micro.txt");

    // surface_mesh.output_surface_mesh(output_file + "_surface" + ".obj");
    // uv_mesh_ptr->output_uv_mesh(output_file + "_uv.obj");
}


// deprecated
// void tetmesh_msh_reader(const std::string& filename, Eigen::MatrixXd& V, Eigen::MatrixXi& T)
// {
//     auto mshio_spec = mshio::load_msh(filename);
//     int DIM = 3;

//     // read vertices
//     mshio::NodeBlock v_block;
//     for (const auto& block : mshio_spec.nodes.entity_blocks) {
//         if (block.entity_dim == DIM) {
//             v_block = block;
//             break;
//         }
//     }

//     const size_t num_vertices = v_block.num_nodes_in_block;
//     V.resize(num_vertices, 3);

//     using Vector = Vector3d;
//     using ConstMapType = typename Vector::ConstMapType;

//     const size_t tag_offset = v_block.tags.front();
//     for (size_t i = 0; i < num_vertices; ++i) {
//         size_t tag = v_block.tags[i] - tag_offset;
//         ConstMapType vec(v_block.data.data() + i * 3);
//         V.row(tag) = vec.head(3).transpose();
//     }

//     // read tets
//     mshio::ElementBlock t_block;
//     for (const auto& block : mshio_spec.elements.entity_blocks) {
//         if (block.entity_dim == DIM) {
//             t_block = block;
//             break;
//         }
//     }

//     const size_t num_elements = t_block.num_elements_in_block;
//     // Eigen::MatrixX<size_t> T;
//     T.resize(num_elements, DIM + 1);

//     const size_t vert_tag_offset = v_block.tags.front();
//     const size_t elem_tag_offset = t_block.data.front();

//     using VectorXi = Eigen::Vector<size_t, 4>;
//     using ConstMapTypei = typename VectorXi::ConstMapType;
//     for (size_t i = 0; i < num_elements; i++) {
//         const size_t tag = t_block.data[i * (DIM + 2)] - elem_tag_offset;
//         assert(tag < num_elements);
//         const size_t* element = t_block.data.data() + i * (DIM + 2) + 1;

//         ConstMapTypei element_vec(element);
//         const auto vertex_tag_offset_vec = VectorXi::Constant(vert_tag_offset);

//         T.row(tag) = (element_vec - vertex_tag_offset_vec).transpose().cast<int>();
//     }
// }


} // namespace wmtk::components::c1_simplification