#include "c1_simplification.hpp"

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include "c1_multimesh.hpp"

#include <jse/jse.h>

#include <memory>
#include <vector>
#include <wmtk/utils/Logger.hpp>

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

    wmtk::logger().set_level(spdlog::level::debug);
    // read surface
    Eigen::MatrixXd s_V, _uv, _N;
    Eigen::MatrixXi s_F, _FT, _FN;
    igl::readOBJ(surface_mesh_file, s_V, _uv, _N, s_F, _FT, _FN);

    // get bbox
    Eigen::Vector3d bbox_max, bbox_min;
    bbox_max = s_V.colwise().maxCoeff();
    bbox_min = s_V.colwise().minCoeff();
    double diag_length = (bbox_max - bbox_min).norm();

    // read uv
    Eigen::MatrixXd _V, uv_V, N;
    Eigen::MatrixXi _F, uv_F, FN;
    igl::readOBJ(uv_mesh_file, _V, uv_V, N, _F, uv_F, FN);

    // TODO: read tetmesh

    std::shared_ptr<MMUVMesh> uv_mesh_ptr = std::make_shared<MMUVMesh>();
    std::shared_ptr<MMTetMesh> tet_mesh_ptr = nullptr;

    uv_mesh_ptr->init_from_eigen(uv_V, uv_F);
    uv_mesh_ptr->output_uv_mesh(output_file + "_uv_initial.obj");
    // TODO: init tetmesh ptr

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

    // TODO: read vertex mapping
    std::map<int64_t, int64_t> s2t_vid_map;


    wmtk::logger().info("meshes and dofs read.");

    MMSurfaceMesh surface_mesh(tet_mesh_ptr, uv_mesh_ptr, threshold * diag_length);
    surface_mesh.init_from_eigen_with_map_and_dofs(s_V, s_F, s2t_vid_map, face_dofs, cone_vids);

    wmtk::logger().info("initialized multimesh. tetmesh involved: {}", tet_mesh_ptr != nullptr);
    surface_mesh.output_tracked_vertices("initial_tracked_vertices.obj");

    int suc_cnt = 0;
    int iter_cnt = 0;
    int try_cnt = 0;
    int total_suc_cnt = 0;

    do {
        suc_cnt = 0;
        auto edge_tuples = surface_mesh.get_edges();
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
                if (surface_mesh.multimesh_collapse_edge(e)) {
                    wmtk::logger().debug("succeeded");
                    // uv_mesh_ptr->output_uv_mesh("uv_" + std::to_string(total_suc_cnt) + ".obj");
                    // surface_mesh.output_tracked_vertices(
                    //     "tracked_vertices" + std::to_string(total_suc_cnt) + ".obj");
                    suc_cnt++;
                    total_suc_cnt++;
                    // break;
                } else if (surface_mesh.multimesh_collapse_edge(e.switch_vertex(surface_mesh))) {
                    wmtk::logger().debug("succeeded other direction");
                    suc_cnt++;
                    total_suc_cnt++;
                } else {
                    wmtk::logger().debug("failed");
                }

                // uv_mesh_ptr->output_uv_mesh(std::to_string(try_cnt) + "_uv.obj", false);
                // try_cnt++;

                // TODO: debug use, remove
                // if (try_cnt > 4) break;
            }
            // if (total_suc_cnt > 0) break;
            // break;
        }


        wmtk::logger().info("iter {} collapse success cnt: {}", iter_cnt, suc_cnt);
        iter_cnt++;

        // if (total_suc_cnt > 0) break;
    } while (suc_cnt > 0);

    wmtk::logger().info("total collapse success cnt: {}", total_suc_cnt);

    surface_mesh.output_surface_mesh(output_file + "_surface" + ".obj");
    uv_mesh_ptr->output_uv_mesh(output_file + "_uv.obj");
}

} // namespace wmtk::components::c1_simplification