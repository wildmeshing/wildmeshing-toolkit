#include "c1_simplification.hpp"
#include <wmtk/io/ComponentIO.hpp>

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include "c1_multimesh.hpp"

#include <jse/jse.h>

#include <memory>
#include <vector>
#include <wmtk/utils/Logger.hpp>

#include <igl/readOBJ.h>
#include <spdlog/common.h>

#include <c1_simplification_spec.hpp>

namespace wmtk::components::c1_simplification {

static void run(nlohmann::json json_params, io::ComponentIO& component_io)
{
    component_io.prepare(json_params, "c1_simplification");
    if (component_io.memory()) {
        json_params.erase("input_dir");
        json_params.erase("report");
        json_params.erase("log_file");
        json_params["tet_mesh"] = "";
        json_params["dofs"] = "";
        json_params["cones"] = "";
    }
    const auto spec = jse::embed::wmtk_c1_simplification_spec::c1_simplification_spec::spec();
    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, spec);
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
    if (component_io.memory()) {
        const auto& data = component_io.mesh(surface_mesh_file);
        io::ComponentIO::attributes(data.vertex_attributes, {"is_cone"}, {"is_cone"});
        for (const auto& [type, c] : data.cells) {
            if (type != CellType::Triangle) throw std::runtime_error("C1 requires triangle meshes");
            io::ComponentIO::attributes(c.attributes, {"dofs"}, {"dofs"});
        }
        s_V = data.V;
        s_F = io::ComponentIO::indices(data.cell(CellType::Triangle).connectivity);
        if (!s_F.rows() || !s_V.rows()) throw std::runtime_error("C1 requires a nonempty surface");
    } else
        igl::readOBJ(surface_mesh_file, s_V, _uv, _N, s_F, _FT, _FN);

    // get bbox
    Eigen::Vector3d bbox_max, bbox_min;
    bbox_max = s_V.colwise().maxCoeff();
    bbox_min = s_V.colwise().minCoeff();
    double diag_length = (bbox_max - bbox_min).norm();

    // read uv
    Eigen::MatrixXd _V, uv_V, N;
    Eigen::MatrixXi _F, uv_F, FN;
    if (component_io.memory()) {
        const auto& data = component_io.mesh(uv_mesh_file);
        io::ComponentIO::attributes(data.vertex_attributes, {});
        for (const auto& [type, c] : data.cells) {
            if (type != CellType::Triangle) throw std::runtime_error("C1 requires UV triangles");
            io::ComponentIO::attributes(c.attributes, {});
        }
        uv_V = data.V.leftCols(2);
        uv_F = io::ComponentIO::indices(data.cell(CellType::Triangle).connectivity);
        if (uv_F.rows() != s_F.rows())
            throw std::runtime_error("C1 UV and surface faces must correspond");
    } else
        igl::readOBJ(uv_mesh_file, _V, uv_V, N, _F, uv_F, FN);

    // TODO: read tetmesh

    std::shared_ptr<MMUVMesh> uv_mesh_ptr = std::make_shared<MMUVMesh>();
    std::shared_ptr<MMTetMesh> tet_mesh_ptr = nullptr;

    uv_mesh_ptr->init_from_eigen(uv_V, uv_F);
    if (!component_io.memory()) uv_mesh_ptr->output_uv_mesh(output_file + "_uv_initial.obj");
    // TODO: init tetmesh ptr

    std::vector<Eigen::Matrix<double, 12, 3>> face_dofs(s_F.rows());
    std::vector<size_t> cone_vids;
    if (component_io.memory()) {
        const auto& data = component_io.mesh(surface_mesh_file);
        const auto& attr = data.cell(CellType::Triangle).attributes.at("dofs");
        if (!std::holds_alternative<DataMatrix<double>>(attr.values) || attr.cols() != 36)
            throw std::runtime_error("C1 dofs must have 36 Float64 columns");
        const auto values = attr.doubles();
        for (int i = 0; i < s_F.rows(); ++i)
            for (int r = 0; r < 12; ++r)
                for (int c = 0; c < 3; ++c) face_dofs[i](r, c) = values(i, r * 3 + c);
        const auto& cone_attr = data.vertex_attributes.at("is_cone");
        if (!std::holds_alternative<DataMatrix<uint8_t>>(cone_attr.values) || cone_attr.cols() != 1)
            throw std::runtime_error("C1 is_cone must have one UInt8 column");
        const auto cones = cone_attr.integers();
        for (int i = 0; i < cones.rows(); ++i) {
            if (cones(i, 0) > 1) throw std::runtime_error("is_cone must be binary");
            if (cones(i, 0)) cone_vids.push_back(i);
        }
    } else {
        std::ifstream dofs(dofs_file);
        for (auto& d : face_dofs)
            for (int r = 0; r < 12; ++r)
                for (int c = 0; c < 3; ++c) dofs >> d(r, c);
        std::ifstream cones(cones_file);
        size_t cid;
        while (cones >> cid) cone_vids.push_back(cid);
    }

    // TODO: read vertex mapping
    std::map<int64_t, int64_t> s2t_vid_map;


    wmtk::logger().info("meshes and dofs read.");

    MMSurfaceMesh surface_mesh(tet_mesh_ptr, uv_mesh_ptr, threshold * diag_length);
    surface_mesh.output_enabled = !component_io.memory();
    surface_mesh.init_from_eigen_with_map_and_dofs(s_V, s_F, s2t_vid_map, face_dofs, cone_vids);

    wmtk::logger().info("initialized multimesh. tetmesh involved: {}", tet_mesh_ptr != nullptr);
    if (!component_io.memory())
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

    if (!component_io.memory()) {
        surface_mesh.output_surface_mesh(output_file + "_surface" + ".obj");
        uv_mesh_ptr->output_uv_mesh(output_file + "_uv.obj");
    } else {
        auto surface =
            io::triangle_data(surface_mesh, [&](size_t v) { return surface_mesh.v_attrs[v].pos; });
        const auto vs = surface_mesh.get_vertices();
        const auto faces = surface_mesh.get_faces();
        DataMatrix<uint8_t> cones(vs.size(), 1);
        for (size_t i = 0; i < vs.size(); ++i)
            cones(i, 0) = surface_mesh.v_attrs[vs[i].vid(surface_mesh)].is_cone;
        surface.vertex_attributes["is_cone"].values = std::move(cones);
        DataMatrix<double> dofs(faces.size(), 36);
        for (size_t i = 0; i < faces.size(); ++i) {
            const auto d = surface_mesh.assemble_dofs(faces[i].fid(surface_mesh));
            for (int r = 0; r < 12; ++r)
                for (int c = 0; c < 3; ++c) dofs(i, r * 3 + c) = d(r, c);
        }
        surface.cells.at(CellType::Triangle).attributes["dofs"].values = std::move(dofs);
        component_io.result.first["mesh"] = std::move(surface);
        component_io.result.first["uv"] =
            io::triangle_data(*uv_mesh_ptr, [&](size_t v) -> Vector3d {
                const auto p = uv_mesh_ptr->v_attrs[v].pos;
                return Vector3d(p[0], p[1], 0);
            });
    }
    component_io.result.second = {{"collapses", total_suc_cnt}, {"iterations", iter_cnt}};
}

void c1_simplification(nlohmann::json options)
{
    io::ComponentIO context;
    run(std::move(options), context);
}
ComponentResult c1_simplification(nlohmann::json options, const MeshMap& meshes)
{
    io::ComponentIO context(&meshes);
    run(std::move(options), context);
    return std::move(context.result);
}

} // namespace wmtk::components::c1_simplification