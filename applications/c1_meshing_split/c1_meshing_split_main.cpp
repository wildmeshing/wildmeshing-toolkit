#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>
#include <wmtk/components/winding_number/winding_number.hpp>

#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>

#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>

#include <fstream>

#include "c1_meshing_split_spec.hpp"

using namespace wmtk;
using namespace wmtk::components;
namespace fs = std::filesystem;

using wmtk::components::utils::resolve_paths;

constexpr static PrimitiveType PV = PrimitiveType::Vertex;
constexpr static PrimitiveType PE = PrimitiveType::Edge;
constexpr static PrimitiveType PF = PrimitiveType::Triangle;
constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

int main(int argc, char* argv[])
{
    CLI::App app{argv[0]};

    app.ignore_case();

    fs::path json_input_file;
    app.add_option("-j, --json", json_input_file, "json specification file")
        ->required(true)
        ->check(CLI::ExistingFile);
    CLI11_PARSE(app, argc, argv);

    nlohmann::json j;
    {
        std::ifstream ifs(json_input_file);
        j = nlohmann::json::parse(ifs);

        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(j, c1_meshing_split_spec);
        if (!r) {
            wmtk::logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, c1_meshing_split_spec);
        }
    }

    fs::path tet_file = resolve_paths(json_input_file, {j["root"], j["tetmesh"]});
    fs::path surface_file = resolve_paths(json_input_file, {j["root"], j["surface_mesh"]});
    fs::path uv_file = resolve_paths(json_input_file, {j["root"], j["uv_mesh"]});
    fs::path map_file = resolve_paths(json_input_file, {j["root"], j["tet_surface_map"]});
    fs::path para_edge_file =
        resolve_paths(json_input_file, {j["root"], j["parametrization_edges"]});
    fs::path cone_edge_file = resolve_paths(json_input_file, {j["root"], j["adjacent_cone_edges"]});
    // fs::path cone_vertices_file = resolve_paths(json_input_file, {j["root"],
    // j["cone_vertices"]});

    auto tetmesh = wmtk::components::input::input(tet_file);
    auto surface_mesh = wmtk::components::input::input(surface_file);
    auto uv_mesh = wmtk::components::input::input(uv_file);

    auto tet_pos_handle = tetmesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto surface_pos_handle =
        surface_mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto uv_pos_handle = uv_mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    auto tet_pos_accessor = tetmesh->create_accessor<double>(tet_pos_handle);
    auto surface_pos_accessor = surface_mesh->create_accessor<double>(surface_pos_handle);

    // register tet -> surface multimesh
    std::ifstream f_map(map_file); // gtid lfid is surface
    std::vector<std::pair<int64_t, int64_t>> tet_surface_map;
    int64_t tid, lfid;
    while (f_map >> tid >> lfid) {
        // std::cout << tid << " " << lfid << std::endl;
        tet_surface_map.emplace_back(tid, lfid);
    }
    f_map.close();
    assert(tet_surface_map.size() > 0);

    auto tets = tetmesh->get_all(PrimitiveType::Tetrahedron);
    auto surfaces = surface_mesh->get_all(PrimitiveType::Triangle);
    assert(surfaces.size() == tet_surface_map.size());

    std::vector<std::array<Tuple, 2>> tet2surface_map_tuples;
    for (int64_t i = 0; i < surfaces.size(); ++i) {
        auto t = tets[tet_surface_map[i].first];
        std::array<Tuple, 4> fs = {
            {tetmesh->switch_tuples(t, {PV, PE, PF}),
             tetmesh->switch_tuples(t, {PE, PF}),
             t,
             tetmesh->switch_tuples(t, {PF})}};

        // fix local ids
        auto tuple = fs[tet_surface_map[i].second];
        // fix vertex
        int try_cnt = 0;
        while ((surface_pos_accessor.const_vector_attribute(surfaces[i]) -
                tet_pos_accessor.const_vector_attribute(tuple))
                   .norm() > 1e-10) {
            if (try_cnt > 3) {
                throw std::runtime_error("cannot find matching vertex tuple!, something is wrong");
            }
            tuple = tetmesh->switch_tuples(tuple, {PV, PE});
            try_cnt++;
        }
        // fix edge
        try_cnt = 0;
        while ((surface_pos_accessor.const_vector_attribute(
                    surface_mesh->switch_tuple(surfaces[i], PV)) -
                tet_pos_accessor.const_vector_attribute(tetmesh->switch_tuple(tuple, PV)))
                   .norm() > 1e-10) {
            if (try_cnt > 2) {
                throw std::runtime_error("cannot find matching edge tuple!, something is wrong");
            }
            tuple = tetmesh->switch_tuple(tuple, PE);
            try_cnt++;
        }
        tet2surface_map_tuples.push_back({{surfaces[i], tuple}});


        // tet2surface_map_tuples.push_back({{fs[tet_surface_map[i].second], surfaces[i]}});
        // tet2surface_map_tuples.push_back({{surfaces[i], fs[tet_surface_map[i].second]}});
    }

    tetmesh->register_child_mesh(surface_mesh, tet2surface_map_tuples);

    // wmtk::components::output::output(*surface_mesh, "test_init_surface_mesh", "vertices");

    // register surface->uv multimesh
    auto surface2uv_map_tuples =
        wmtk::multimesh::same_simplex_dimension_bijection(*surface_mesh, *uv_mesh);
    surface_mesh->register_child_mesh(uv_mesh, surface2uv_map_tuples);

    // initial vertex id attribute
    auto vid_handle = surface_mesh->register_attribute<int64_t>("vid", PrimitiveType::Vertex, 1);
    auto vid_accessor = surface_mesh->create_accessor<int64_t>(vid_handle);

    auto surface_vertices = surface_mesh->get_all(PrimitiveType::Vertex);
    for (int64_t i = 0; i < surface_vertices.size(); ++i) {
        vid_accessor.scalar_attribute(surface_vertices[i]) = i;
    }

    int cnt = 0;
    for (const auto& v : surface_mesh->get_all(PrimitiveType::Vertex)) {
        auto surface_coord = surface_pos_accessor.const_vector_attribute(v);
        auto tv = surface_mesh->map_to_parent_tuple(wmtk::simplex::Simplex::vertex(v));
        auto tet_coord = tet_pos_accessor.const_vector_attribute(tv);

        if ((surface_coord - tet_coord).norm() > 1e-8) {
            wmtk::logger().error(
                "{} surface and tet position not match {} | {}",
                cnt,
                surface_coord.transpose(),
                tet_coord.transpose());
        }
        cnt++;
    }

    wmtk::operations::EdgeSplit split_op(*surface_mesh);
    split_op.set_new_attribute_strategy(tet_pos_handle);
    split_op.set_new_attribute_strategy(surface_pos_handle);
    split_op.set_new_attribute_strategy(uv_pos_handle);
    split_op.set_new_attribute_strategy(
        vid_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);


    // para edge split
    std::ifstream f_para_edges(para_edge_file);
    std::vector<std::array<int64_t, 2>> para_edges;
    int64_t a, b;
    while (f_para_edges >> a >> b) {
        para_edges.push_back({{a, b}});
    }

    if (para_edges.size() > 0) {
        throw std::runtime_error("Do not support para edge");
    }

    // for (int64_t i = 0; i < para_edges.size(); ++i) {
    //     const int64_t v0 = para_edges[0];
    //     const int64_t v1 = para_edges[1];
    //     for (auto e : surface_mesh->get_all(PrimitiveType::Edge)) {
    //         int64_t ev0 = vid_accessor.const_scalar_attribute(e);
    //         int64_t ev1 = vid_accessor.const_scalar_attribute(
    //             surface_mesh->switch_tuple(PrimitiveType::Vertex));

    //         if ((ev0 == v0 && ev1 == v1) || (ev0 == v1 && ev1 == v0)) {
    //             auto new_v_tuple = split_op(e).tuple();
    //             vid_accessor.scalar_attribute(new_v_tuple) = surface_vertices.size();
    //         }
    //     }
    // }

    // cone split edge
    std::ifstream f_cone_edges(cone_edge_file);
    std::vector<std::array<int64_t, 2>> cone_edges;
    while (f_cone_edges >> a >> b) {
        cone_edges.push_back({{a, b}});
    }

    int64_t v_cnt = surface_mesh->get_all(PrimitiveType::Vertex).size();
    std::vector<std::array<int64_t, 2>> new_cone_edges;
    for (int64_t i = 0; i < cone_edges.size(); ++i) {
        const int64_t v0 = cone_edges[i][0];
        const int64_t v1 = cone_edges[i][1];
        wmtk::logger().info("Splitting the edge {} {}", v0, v1);
        for (auto e : surface_mesh->get_all(PrimitiveType::Edge)) {
            int64_t ev0 = vid_accessor.const_scalar_attribute(e);
            int64_t ev1 = vid_accessor.const_scalar_attribute(
                surface_mesh->switch_tuple(e, PrimitiveType::Vertex));
            // std::cout << ev0 << ", " << ev1 << std::endl;

            if ((ev0 == v0 && ev1 == v1) || (ev0 == v1 && ev1 == v0)) {
                auto new_v_tuple =
                    split_op(wmtk::simplex::Simplex::edge(*surface_mesh, e)).front().tuple();
                vid_accessor.scalar_attribute(new_v_tuple) = v_cnt;
                new_cone_edges.push_back({{v_cnt, v0}});
                new_cone_edges.push_back({{v_cnt, v1}});
                v_cnt++;

                break;
            }
        }
    }

    wmtk::logger().info("2nd split edges");
    for (int64_t i = 0; i < new_cone_edges.size(); ++i) {
        const int64_t v0 = new_cone_edges[i][0];
        const int64_t v1 = new_cone_edges[i][1];
        wmtk::logger().info("Splitting the edge {} {}", v0, v1);
        for (auto e : surface_mesh->get_all(PrimitiveType::Edge)) {
            int64_t ev0 = vid_accessor.const_scalar_attribute(e);
            int64_t ev1 = vid_accessor.const_scalar_attribute(
                surface_mesh->switch_tuple(e, PrimitiveType::Vertex));
            // std::cout << ev0 << ", " << ev1 << std::endl;

            if ((ev0 == v0 && ev1 == v1) || (ev0 == v1 && ev1 == v0)) {
                auto new_v_tuple =
                    split_op(wmtk::simplex::Simplex::edge(*surface_mesh, e)).front().tuple();
                vid_accessor.scalar_attribute(new_v_tuple) = v_cnt;
                v_cnt++;

                break;
            }
        }
    }


    // std::ifstream f_cone_vertices(cone_vertices_file);
    // std::vector<int64_t> cone_vertices;
    // while (f_cone_vertices >> a) {
    //     cone_edges.push_back(a);
    // }

    // for (int pass = 0; pass < 3; ++pass) {
    //     auto cmp = [](const std::pair<Tuple, double>& a, const std::pair<Tuple, double>& b) {
    //         return a.second < b.second;
    //     };
    //     std::priority_queue<std::pair<Tuple, double>, std::vector<std::pair<Tuple, double>>, cmp>
    //         tuple_length;
    //     for (const auto& e : surface_mesh->get_all(PrimitiveType::Edge)) {
    //         int64_t ev0 = vid_accessor.const_scalar_attribute(e);
    //         int64_t ev1 = vid_accessor.const_scalar_attribute(
    //             surface_mesh->switch_tuple(e, PrimitiveType::Vertex));
    //         if (std::find(cone_vertices.begin(), cone_vertices.end(), ev0) != cone_vertices.end()
    //         ||
    //             std::find(cone_vertices.begin(), cone_vertices.end(), ev1) !=
    //             cone_vertices.end()) { double length =
    //             surface_pos_accessor.const_vector_attribute(e) -
    //                             surface_pos_accessor
    //                                 .const_vector_attribute(
    //                                     surface_mesh->switch_tuple(e, PrimitiveType::Vertex))
    //                                 .norm();
    //             tuple_length.emplace(e, length);
    //         }
    //     }
    // }

    // consolidate the mesh
    wmtk::multimesh::consolidate(*tetmesh);

    // register final ids
    auto surface_final_vid_handle =
        surface_mesh->register_attribute<int64_t>("final_vid", PrimitiveType::Vertex, 1);
    auto tet_final_vid_handle =
        tetmesh->register_attribute<int64_t>("final_vid", PrimitiveType::Vertex, 1);
    auto tet_final_tid_handle =
        tetmesh->register_attribute<int64_t>("final_tid", PrimitiveType::Tetrahedron, 1);
    auto uv_final_fid_handle =
        uv_mesh->register_attribute<int64_t>("final_fid", PrimitiveType::Triangle, 1);
    auto uv_final_vid_handle =
        uv_mesh->register_attribute<int64_t>("final_vid", PrimitiveType::Vertex, 1);

    auto surface_final_vid_accessor =
        surface_mesh->create_accessor<int64_t>(surface_final_vid_handle);
    auto tet_final_vid_accessor = tetmesh->create_accessor<int64_t>(tet_final_vid_handle);
    auto tet_final_tid_accessor = tetmesh->create_accessor<int64_t>(tet_final_tid_handle);
    auto uv_final_fid_accessor = uv_mesh->create_accessor<int64_t>(uv_final_fid_handle);
    auto uv_final_vid_accessor = uv_mesh->create_accessor<int64_t>(uv_final_vid_handle);

    const auto& tet_final_vertices = tetmesh->get_all(PrimitiveType::Vertex);
    for (int64_t i = 0; i < tet_final_vertices.size(); ++i) {
        tet_final_vid_accessor.scalar_attribute(tet_final_vertices[i]) = i;
    }

    const auto& tet_final_tets = tetmesh->get_all(PrimitiveType::Tetrahedron);
    for (int64_t i = 0; i < tet_final_tets.size(); ++i) {
        tet_final_tid_accessor.scalar_attribute(tet_final_tets[i]) = i;
    }

    const auto& uv_final_faces = uv_mesh->get_all(PrimitiveType::Triangle);
    for (int64_t i = 0; i < uv_final_faces.size(); ++i) {
        uv_final_fid_accessor.scalar_attribute(uv_final_faces[i]) = i;
    }

    const auto& surface_final_vertices = surface_mesh->get_all(PrimitiveType::Vertex);
    for (int64_t i = 0; i < surface_final_vertices.size(); ++i) {
        surface_final_vid_accessor.scalar_attribute(surface_final_vertices[i]) = i;
    }

    const auto& uv_final_vertices = uv_mesh->get_all(PrimitiveType::Vertex);
    for (int64_t i = 0; i < uv_final_vertices.size(); ++i) {
        uv_final_vid_accessor.scalar_attribute(uv_final_vertices[i]) = i;
    }

    // get surface -> tet map
    std::vector<std::array<int64_t, 3>> surface_adj_tets; // surface id, tid0, tid1
    const auto& surface_final_faces = surface_mesh->get_all(PrimitiveType::Triangle);
    for (int64_t i = 0; i < surface_final_faces.size(); ++i) {
        auto t0 =
            surface_mesh->map_to_parent_tuple(wmtk::simplex::Simplex::face(surface_final_faces[i]));
        assert(!tetmesh->is_boundary(PrimitiveType::Triangle, t0));
        auto t1 = tetmesh->switch_tuple(t0, PrimitiveType::Tetrahedron);

        surface_adj_tets.push_back(
            {{i,
              tet_final_tid_accessor.const_scalar_attribute(t0),
              tet_final_tid_accessor.const_scalar_attribute(t1)}});
    }

    // get surface v -> tet v map
    std::vector<int64_t> surface_v_to_tet_v; // surface vid, tet vid
    // const auto& surface_final_vertices = surface_mesh->get_all(PrimitiveType::Vertex);
    for (int64_t i = 0; i < surface_final_vertices.size(); ++i) {
        auto tv = surface_mesh->map_to_parent_tuple(
            wmtk::simplex::Simplex::vertex(surface_final_vertices[i]));

        surface_v_to_tet_v.push_back(tet_final_vid_accessor.const_scalar_attribute(tv));
    }

    // get surface f -> uv f map
    std::vector<int64_t> surface_f_to_uv_f;
    for (int64_t i = 0; i < surface_final_faces.size(); ++i) {
        auto fuv = surface_mesh
                       ->map_to_child_tuples(
                           *uv_mesh,
                           wmtk::simplex::Simplex::face(surface_final_faces[i]))
                       .front();

        surface_f_to_uv_f.push_back(uv_final_fid_accessor.const_scalar_attribute(fuv));
    }

    // write surface -> tet map
    std::ofstream surface_adj_tet_file("surface_adj_tet_after_cone_split.txt");
    for (const auto& l : surface_adj_tets) {
        surface_adj_tet_file << l[0] << " " << l[1] << " " << l[2] << std::endl;
    }
    surface_adj_tet_file.close();

    // write surface v to tet v map
    std::ofstream surface_v_to_tet_v_file("surface_v_to_tet_v_after_cone_split.txt");
    for (const auto& l : surface_v_to_tet_v) {
        surface_v_to_tet_v_file << l << std::endl;
    }
    surface_v_to_tet_v_file.close();

    // write obj file
    wmtk::utils::EigenMatrixWriter surface_writer;
    surface_mesh->serialize(surface_writer);
    Eigen::MatrixX<int64_t> surface_f;
    Eigen::MatrixXd surface_v;
    surface_writer.get_FV_matrix(surface_f);
    surface_writer.get_position_matrix(surface_v);

    wmtk::utils::EigenMatrixWriter uv_writer;
    uv_mesh->serialize(uv_writer);
    Eigen::MatrixX<int64_t> uv_f;
    Eigen::MatrixXd uv_v;
    uv_writer.get_FV_matrix(uv_f);
    uv_writer.get_position_matrix(uv_v);

    std::ofstream obj_file("surface_uv_after_cone_split.obj");
    for (int64_t i = 0; i < surface_v.rows(); ++i) {
        obj_file << std::setprecision(17) << "v " << surface_v(i, 0) << " " << surface_v(i, 1)
                 << " " << surface_v(i, 2) << std::endl;
    }
    for (int64_t i = 0; i < uv_v.rows(); ++i) {
        obj_file << std::setprecision(17) << "vt " << uv_v(i, 0) << " " << uv_v(i, 1) << std::endl;
    }
    obj_file << std::endl;
    // for (int64_t i = 0; i < surface_f.rows(); ++i) {
    //     const auto& sf = surface_f.row(i);
    //     const auto& uvf = uv_f.row(surface_f_to_uv_f[i]);
    //     obj_file << "f " << sf[0] + 1 << "/" << uvf[0] + 1 << " " << sf[1] + 1 << "/" << uvf[1] + 1
    //              << " " << sf[2] + 1 << "/" << uvf[2] + 1 << std::endl;
    // }
    for (const auto& f : surface_mesh->get_all(PrimitiveType::Triangle)) {
        auto s_v0 = surface_final_vid_accessor.const_scalar_attribute(f);
        auto s_v1 =
            surface_final_vid_accessor.const_scalar_attribute(surface_mesh->switch_tuple(f, PV));
        auto s_v2 = surface_final_vid_accessor.const_scalar_attribute(
            surface_mesh->switch_tuples(f, {PE, PV}));

        auto f_uv =
            surface_mesh->map_to_child_tuples(*uv_mesh, wmtk::simplex::Simplex::face(f)).front();
        auto uv_v0 = uv_final_vid_accessor.const_scalar_attribute(f_uv);
        auto uv_v1 = uv_final_vid_accessor.const_scalar_attribute(uv_mesh->switch_tuple(f_uv, PV));
        auto uv_v2 =
            uv_final_vid_accessor.const_scalar_attribute(uv_mesh->switch_tuples(f_uv, {PE, PV}));

        obj_file << "f " << s_v0 + 1 << "/" << uv_v0 + 1 << " " << s_v1 + 1 << "/" << uv_v1 + 1
                 << " " << s_v2 + 1 << "/" << uv_v2 + 1 << std::endl;
    }

    obj_file.close();


    // winding number
    auto mesh_after_winding_number = winding_number(tetmesh, surface_mesh);

    std::string output = j["output"];
    wmtk::components::output::output(*mesh_after_winding_number, output + "_tetmesh", "vertices");
    wmtk::components::output::output(*surface_mesh, output + "_surface_mesh", "vertices");
    wmtk::components::output::output(*uv_mesh, output + "_uv_mesh", "vertices");


    const std::string report = j["report"];
    if (!report.empty()) {
        nlohmann::json out_json;
        out_json["vertices"] = mesh_after_winding_number->get_all(PrimitiveType::Vertex).size();
        out_json["edges"] = mesh_after_winding_number->get_all(PrimitiveType::Edge).size();
        out_json["faces"] = mesh_after_winding_number->get_all(PrimitiveType::Triangle).size();
        out_json["cells"] = mesh_after_winding_number->get_all(PrimitiveType::Tetrahedron).size();

        out_json["input"] = j;

        std::ofstream ofs(report);
        ofs << out_json;
    }


    return 0;
}