#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/utils/resolve_path.hpp>
#include <wmtk/components/winding_number/winding_number.hpp>

#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>

#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>

#include <igl/readOBJ.h>

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

bool match_F(
    const std::array<int64_t, 3>& surface_f,
    const std::array<int64_t, 3>& uv_f,
    std::array<int64_t, 3>& local_sf_in_uv_ids)
{
    for (int i = 0; i < 3; ++i) {
        bool found = false;
        for (int j = 0; j < 3; ++j) {
            if (surface_f[i] == uv_f[j]) {
                local_sf_in_uv_ids[i] = j;
                found = true;
                break;
            }
        }
        if (!found) {
            return false;
        }
    }

    return true;
}

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
    // fs::path uv_file = resolve_paths(json_input_file, {j["root"], j["uv_mesh"]});
    fs::path map_file = resolve_paths(json_input_file, {j["root"], j["tet_surface_map"]});
    fs::path para_edge_file =
        resolve_paths(json_input_file, {j["root"], j["parametrization_edges"]});
    // fs::path cone_edge_file =
    //     resolve_paths(json_input_file, {j["root"], j["adjacent_para_edges_with_v"]});

    fs::path cone_vertices_file = resolve_paths(json_input_file, {j["root"], j["cone_vertices"]});
    std::ifstream cone_vs(cone_vertices_file);
    std::vector<int64_t> cone_vids;
    int64_t cone_vid;
    while (cone_vs >> cone_vid) {
        cone_vids.push_back(cone_vid);
    }


    // read uv obj file
    fs::path uv_obj_file = resolve_paths(json_input_file, {j["root"], j["uv_mesh"]});
    Eigen::MatrixXd V, TC, CN;
    Eigen::MatrixXi F, FTC, FN;
    igl::readOBJ(uv_obj_file, V, TC, CN, F, FTC, FN);

    // read input
    auto tetmesh = wmtk::components::input::input(tet_file);
    auto surface_mesh = wmtk::components::input::input(surface_file);
    // auto uv_mesh = wmtk::components::input::input(uv_file);

    auto tet_pos_handle = tetmesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto surface_pos_handle =
        surface_mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    // auto uv_pos_handle = uv_mesh->get_attribute_handle<double>("vertices",
    // PrimitiveType::Vertex);

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

    // initialize cone tag attribute
    auto cone_handle = surface_mesh->register_attribute<int64_t>("cone", PrimitiveType::Vertex, 1);
    auto cone_accessor = surface_mesh->create_accessor<int64_t>(cone_handle);

    const auto& sf_vs = surface_mesh->get_all(PrimitiveType::Vertex);
    for (const auto& v : sf_vs) {
        cone_accessor.scalar_attribute(v) = 0;
    }

    for (int64_t i = 0; i < cone_vids.size(); ++i) {
        cone_accessor.scalar_attribute(sf_vs[cone_vids[i]]) = 1;
    }


    wmtk::operations::EdgeSplit split_op(*surface_mesh);
    split_op.set_new_attribute_strategy(tet_pos_handle);
    split_op.set_new_attribute_strategy(surface_pos_handle);
    split_op.set_new_attribute_strategy(
        vid_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    split_op.set_new_attribute_strategy(
        cone_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);


    // para edge split
    std::ifstream f_para_edges(para_edge_file);
    std::vector<std::array<int64_t, 3>> para_edges_with_v; // {{ev0, ev1, split_vid}}
    int64_t a, b, c;
    while (f_para_edges >> a >> b >> c) {
        para_edges_with_v.push_back({{a, b, c}});
    }

    int64_t v_cnt = surface_mesh->get_all(PrimitiveType::Vertex).size();
    // std::vector<std::array<int64_t, 2>> new_para_edges_with_v;
    for (int64_t i = 0; i < para_edges_with_v.size(); ++i) {
        const int64_t v0 = para_edges_with_v[i][0];
        const int64_t v1 = para_edges_with_v[i][1];
        const int64_t v_split_id = para_edges_with_v[i][2];
        wmtk::logger().info("Splitting the edge {} {}", v0, v1);
        for (const auto& e : surface_mesh->get_all(PrimitiveType::Edge)) {
            int64_t ev0 = vid_accessor.const_scalar_attribute(e);
            int64_t ev1 = vid_accessor.const_scalar_attribute(
                surface_mesh->switch_tuple(e, PrimitiveType::Vertex));
            // std::cout << ev0 << ", " << ev1 << std::endl;

            if ((ev0 == v0 && ev1 == v1) || (ev0 == v1 && ev1 == v0)) {
                auto new_v_tuple =
                    split_op(wmtk::simplex::Simplex::edge(*surface_mesh, e)).front().tuple();

                // set split vid to be the same as para after file
                vid_accessor.scalar_attribute(new_v_tuple) = v_split_id;
                cone_accessor.scalar_attribute(new_v_tuple) = 0;
                v_cnt++;

                break;
            }
        }
    }

    // consolidate the mesh
    wmtk::multimesh::consolidate(*tetmesh);

    std::cout << "Done para split" << std::endl;

    // register final ids
    auto surface_final_vid_handle =
        surface_mesh->register_attribute<int64_t>("final_vid", PrimitiveType::Vertex, 1);
    auto tet_final_vid_handle =
        tetmesh->register_attribute<int64_t>("final_vid", PrimitiveType::Vertex, 1);
    auto tet_final_tid_handle =
        tetmesh->register_attribute<int64_t>("final_tid", PrimitiveType::Tetrahedron, 1);

    auto surface_final_vid_accessor =
        surface_mesh->create_accessor<int64_t>(surface_final_vid_handle);
    auto tet_final_vid_accessor = tetmesh->create_accessor<int64_t>(tet_final_vid_handle);
    auto tet_final_tid_accessor = tetmesh->create_accessor<int64_t>(tet_final_tid_handle);

    const auto& tet_final_vertices = tetmesh->get_all(PrimitiveType::Vertex);
    for (int64_t i = 0; i < tet_final_vertices.size(); ++i) {
        tet_final_vid_accessor.scalar_attribute(tet_final_vertices[i]) = i;
    }

    const auto& tet_final_tets = tetmesh->get_all(PrimitiveType::Tetrahedron);
    for (int64_t i = 0; i < tet_final_tets.size(); ++i) {
        tet_final_tid_accessor.scalar_attribute(tet_final_tets[i]) = i;
    }

    const auto& surface_final_vertices = surface_mesh->get_all(PrimitiveType::Vertex);
    for (int64_t i = 0; i < surface_final_vertices.size(); ++i) {
        assert(vid_accessor.const_scalar_attribute(surface_final_vertices[i]) > -1);
        surface_final_vid_accessor.scalar_attribute(surface_final_vertices[i]) =
            vid_accessor.const_scalar_attribute(surface_final_vertices[i]);
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

    // get tet face to surface map (fid, tid, lfid)
    std::vector<std::array<int64_t, 3>> surface_to_tet_local_face_final_map;
    for (int64_t i = 0; i < surface_final_faces.size(); ++i) {
        const auto& t =
            surface_mesh->map_to_parent_tuple(wmtk::simplex::Simplex::face(surface_final_faces[i]));
        int64_t lfid = wmtk::utils::TupleInspector::local_fid(t);
        surface_to_tet_local_face_final_map.push_back(
            {{i, tet_final_tid_accessor.const_scalar_attribute(t), lfid}});
    }

    // get surface v -> tet v map
    std::vector<int64_t> surface_v_to_tet_v(
        surface_final_vertices.size(),
        -1); // surface vid, tet vid

    for (int64_t i = 0; i < surface_final_vertices.size(); ++i) {
        auto tv = surface_mesh->map_to_parent_tuple(
            wmtk::simplex::Simplex::vertex(surface_final_vertices[i]));

        surface_v_to_tet_v[surface_final_vid_accessor.const_scalar_attribute(
            surface_final_vertices[i])] = tet_final_vid_accessor.const_scalar_attribute(tv);
    }

    // match surface f with uv f
    wmtk::utils::EigenMatrixWriter surface_writer;
    surface_mesh->serialize(surface_writer);
    Eigen::MatrixX<int64_t> surface_f;
    Eigen::MatrixXd surface_v;
    surface_writer.get_FV_matrix(surface_f);
    surface_writer.get_position_matrix(surface_v);

    // get final surface v and f
    Eigen::MatrixXd surface_v_final(surface_v.rows(), surface_v.cols());
    for (int64_t i = 0; i < surface_final_vertices.size(); ++i) {
        const auto& vid_final =
            surface_final_vid_accessor.const_scalar_attribute(surface_final_vertices[i]);
        surface_v_final.row(vid_final) = surface_v.row(i);
    }

    Eigen::MatrixX<int64_t> surface_f_final = surface_f;
    for (int64_t i = 0; i < surface_f_final.rows(); ++i) {
        for (int64_t j = 0; j < surface_f_final.cols(); ++j) {
            surface_f_final(i, j) = surface_final_vid_accessor.const_scalar_attribute(
                surface_final_vertices[surface_f(i, j)]);
        }
    }

    // match f and uv
    assert(surface_f_final.rows() == FTC.rows());
    assert(surface_f_final.rows() == F.rows());

    std::vector<bool> matched_F(F.rows(), false);
    Eigen::MatrixX<int64_t> uv_f_final(surface_f_final.rows(), surface_f_final.cols());

    for (int64_t i = 0; i < surface_f_final.rows(); ++i) {
        bool find_match = false;
        for (int64_t j = 0; j < F.rows(); ++j) {
            if (matched_F[j]) {
                continue;
            }

            std::array<int64_t, 3> local_sf_in_uv_ids;

            const std::array<int64_t, 3> sf_row_i = {
                {surface_f_final(i, 0), surface_f_final(i, 1), surface_f_final(i, 2)}};
            const std::array<int64_t, 3> f_row_j = {{F(j, 0), F(j, 1), F(j, 2)}};

            if (match_F(sf_row_i, f_row_j, local_sf_in_uv_ids)) {
                // std::cout << sf_row_i[0] << " " << sf_row_i[1] << " " << sf_row_i[2] << " "
                //           << std::endl;
                // std::cout << f_row_j[0] << " " << f_row_j[1] << " " << f_row_j[2] << " "
                //           << std::endl;
                // std::cout << FTC(j, 0) << " " << FTC(j, 1) << " " << FTC(j, 2) << " " <<
                // std::endl; std::cout << local_sf_in_uv_ids[0] << " " << local_sf_in_uv_ids[1] <<
                // " "
                //           << local_sf_in_uv_ids[2] << " " << std::endl;

                // exit(0);

                uv_f_final(i, 0) = FTC(j, local_sf_in_uv_ids[0]);
                uv_f_final(i, 1) = FTC(j, local_sf_in_uv_ids[1]);
                uv_f_final(i, 2) = FTC(j, local_sf_in_uv_ids[2]);

                // std::cout << uv_f_final.row(i) << std::endl;

                matched_F[j] = true;
                find_match = true;
                break;
            }
        }
        assert(find_match);
    }

    // write tet face to surface map (fid, tid, lfid) file
    std::ofstream tet_surface_map_file("surface_tet_local_face_map_after_para_split.txt");
    for (const auto& l : surface_to_tet_local_face_final_map) {
        tet_surface_map_file << l[1] << " " << l[2] << std::endl;
    }
    tet_surface_map_file.close();

    // write cone vids
    std::ofstream cone_vids_file("cone_vids_after_para_split.txt");
    for (const auto& v : surface_mesh->get_all(PrimitiveType::Vertex)) {
        if (cone_accessor.const_scalar_attribute(v) == 1) {
            cone_vids_file << surface_final_vid_accessor.const_scalar_attribute(v) << std::endl;
        }
    }


    // write surface -> tet map
    std::ofstream surface_adj_tet_file("surface_adj_tet_after_para_split.txt");
    for (const auto& l : surface_adj_tets) {
        surface_adj_tet_file << l[0] << " " << l[1] << " " << l[2] << std::endl;
    }
    surface_adj_tet_file.close();

    // write surface v to tet v map
    std::ofstream surface_v_to_tet_v_file("surface_v_to_tet_v_after_para_split.txt");
    for (const auto& l : surface_v_to_tet_v) {
        surface_v_to_tet_v_file << l << std::endl;
    }
    surface_v_to_tet_v_file.close();

    // write obj file

    std::ofstream obj_file("surface_uv_after_para_split.obj");
    for (int64_t i = 0; i < surface_v_final.rows(); ++i) {
        obj_file << std::setprecision(17) << "v " << surface_v_final(i, 0) << " "
                 << surface_v_final(i, 1) << " " << surface_v_final(i, 2) << std::endl;
    }
    for (int64_t i = 0; i < TC.rows(); ++i) {
        obj_file << std::setprecision(17) << "vt " << TC(i, 0) << " " << TC(i, 1) << std::endl;
    }
    obj_file << std::endl;

    for (int64_t i = 0; i < surface_f_final.rows(); ++i) {
        int64_t s_v0 = surface_f_final(i, 0);
        int64_t s_v1 = surface_f_final(i, 1);
        int64_t s_v2 = surface_f_final(i, 2);

        int64_t uv_v0 = uv_f_final(i, 0);
        int64_t uv_v1 = uv_f_final(i, 1);
        int64_t uv_v2 = uv_f_final(i, 2);

        obj_file << "f " << s_v0 + 1 << "/" << uv_v0 + 1 << " " << s_v1 + 1 << "/" << uv_v1 + 1
                 << " " << s_v2 + 1 << "/" << uv_v2 + 1 << std::endl;
    }

    obj_file.close();

    // winding number
    auto mesh_after_winding_number = winding_number(tetmesh, surface_mesh);

    std::string output = j["output"];
    wmtk::components::output::output(*mesh_after_winding_number, output + "_tetmesh", "vertices");
    // wmtk::components::output::output(*surface_mesh, output + "_surface_mesh", "vertices");

    std::cout << "File written" << std::endl;

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