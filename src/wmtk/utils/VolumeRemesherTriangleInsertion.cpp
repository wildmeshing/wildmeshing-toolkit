
#include "VolumeRemesherTriangleInsertion.hpp"
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk/utils/orient.hpp>


// clang-format off
#include <VolumeRemesher/embed.h>
// clang-format on

#include <numeric>

#include <polysolve/Utils.hpp>


namespace wmtk::utils {

std::tuple<std::shared_ptr<wmtk::TetMesh>, std::vector<std::array<bool, 4>>>
generate_raw_tetmesh_from_input_surface(
    const RowVectors3d& V,
    const RowVectors3l& F,
    const RowVectors3d& background_V,
    const RowVectors4l& background_TV)
{
    polysolve::StopWatch timer("tri insertion", logger());

    // prepare data for volume remesher
    // inputs
    std::vector<double> tri_vrt_coords(V.rows() * 3);
    std::vector<uint32_t> triangle_indices(F.rows() * 3);
    std::vector<double> tet_vrt_coords(background_V.rows() * 3);
    std::vector<uint32_t> tet_indices(background_TV.rows() * 4);

    for (int64_t i = 0; i < V.rows(); ++i) {
        tri_vrt_coords[i * 3 + 0] = V(i, 0);
        tri_vrt_coords[i * 3 + 1] = V(i, 1);
        tri_vrt_coords[i * 3 + 2] = V(i, 2);
    }

    for (int64_t i = 0; i < F.rows(); ++i) {
        triangle_indices[i * 3 + 0] = F(i, 0);
        triangle_indices[i * 3 + 1] = F(i, 1);
        triangle_indices[i * 3 + 2] = F(i, 2);
    }

    for (int64_t i = 0; i < background_V.rows(); ++i) {
        tet_vrt_coords[i * 3 + 0] = background_V(i, 0);
        tet_vrt_coords[i * 3 + 1] = background_V(i, 1);
        tet_vrt_coords[i * 3 + 2] = background_V(i, 2);
    }

    for (int64_t i = 0; i < background_TV.rows(); ++i) {
        tet_indices[i * 4 + 0] = background_TV(i, 0);
        tet_indices[i * 4 + 1] = background_TV(i, 1);
        tet_indices[i * 4 + 2] = background_TV(i, 2);
        tet_indices[i * 4 + 3] = background_TV(i, 3);
    }

    // outputs
    std::vector<vol_rem::bigrational> embedded_vertices;
    std::vector<uint32_t> embedded_facets;
    std::vector<uint32_t> embedded_cells;
    std::vector<uint32_t> embedded_facets_on_input;
    std::vector<std::array<uint32_t, 4>> tets_final; // final tets
    std::vector<uint32_t> final_tets_parent;
    std::vector<bool> cells_with_faces_on_input;
    std::vector<std::vector<uint32_t>> final_tets_parent_faces;
    timer.start();
    // run volume remesher
    vol_rem::embed_tri_in_poly_mesh(
        tri_vrt_coords,
        triangle_indices,
        tet_vrt_coords,
        tet_indices,
        embedded_vertices,
        embedded_facets,
        embedded_cells,
        tets_final,
        final_tets_parent,
        embedded_facets_on_input,
        cells_with_faces_on_input,
        final_tets_parent_faces,
        wmtk::logger().level() < spdlog::level::info);
    timer.stop();
    assert(tets_final.size() == final_tets_parent.size());

    wmtk::logger().info(
        "volume remesher finished {}s, polycell mesh generated, #vertices: {},  #cells: {}, #tets "
        "{}",
        timer.getElapsedTimeInSec(),
        embedded_vertices.size() / 3,
        embedded_cells.size(),
        tets_final.size());

    // convert to double and readable format
    std::vector<Vector3r> v_coords; // vertex coordinates
    std::vector<std::array<int64_t, 3>> polygon_faces; // index of vertices
    std::vector<bool> polygon_faces_on_input; // whether the face is on input surface
    std::vector<std::array<bool, 4>> tet_face_on_input_surface; // tet local face on input surface


    wmtk::logger().info("Converting vertices...");
    int64_t round_cnt = 0;
    for (int64_t i = 0; i < embedded_vertices.size() / 3; ++i) {
#ifdef USE_GNU_GMP_CLASSES
        v_coords.emplace_back();
        v_coords.back()[0].init(embedded_vertices[3 * i + 0].get_mpq_t());
        v_coords.back()[1].init(embedded_vertices[3 * i + 1].get_mpq_t());
        v_coords.back()[2].init(embedded_vertices[3 * i + 2].get_mpq_t());
#else
        v_coords.emplace_back();
        v_coords.back()[0].init_from_binary(embedded_vertices[3 * i + 0].get_str());
        v_coords.back()[1].init_from_binary(embedded_vertices[3 * i + 1].get_str());
        v_coords.back()[2].init_from_binary(embedded_vertices[3 * i + 2].get_str());
#endif


        bool round = true;
        for (int64_t j = 0; j < 3; ++j) {
            if (!v_coords.back()[j].can_be_rounded()) {
                round = false;
                break;
            }
        }

        if (round) {
            round_cnt++;
            for (int64_t j = 0; j < 3; ++j) v_coords.back()[j].round();
        }
    }
    wmtk::logger().info("done, {} rounded vertices over {}", round_cnt, v_coords.size());

    wmtk::logger().info("Facets loop...");
    for (int64_t i = 0; i < embedded_facets.size(); ++i) {
        int64_t polysize = embedded_facets[i];
        assert(polysize == 3);
        std::array<int64_t, 3> polygon;
        for (int64_t j = i + 1; j <= i + polysize; ++j) {
            polygon[j - (i + 1)] = embedded_facets[j];
        }
        polygon_faces.push_back(polygon);
        i += polysize; // dangerous behavior, but working now
    }
    wmtk::logger().info("done");

    wmtk::logger().info("Tags loop...");
    polygon_faces_on_input.resize(polygon_faces.size(), false);
    for (int64_t i = 0; i < embedded_facets_on_input.size(); ++i) {
        polygon_faces_on_input[embedded_facets_on_input[i]] = true;
    }
    wmtk::logger().info("done");


    wmtk::logger().info("tracking surface starting...");
    timer.start();
    assert(final_tets_parent_faces.size() == tets_final.size());
    for (int64_t i = 0; i < tets_final.size(); ++i) {
        const auto& tetra = tets_final[i];
        const uint32_t tetra_parent = final_tets_parent[i];

        if (!cells_with_faces_on_input[tetra_parent]) {
            tet_face_on_input_surface.push_back({false, false, false, false});
            continue;
        }

        // vector of std array and sort
        std::array<int64_t, 3> local_f0{{tetra[1], tetra[2], tetra[3]}};
        std::sort(local_f0.begin(), local_f0.end());
        std::array<int64_t, 3> local_f1{{tetra[0], tetra[2], tetra[3]}};
        std::sort(local_f1.begin(), local_f1.end());
        std::array<int64_t, 3> local_f2{{tetra[0], tetra[1], tetra[3]}};
        std::sort(local_f2.begin(), local_f2.end());
        std::array<int64_t, 3> local_f3{{tetra[0], tetra[1], tetra[2]}};
        std::sort(local_f3.begin(), local_f3.end());

        // track surface
        std::array<bool, 4> tet_face_on_input{{false, false, false, false}};
        for (auto f : final_tets_parent_faces[i]) {
            assert(polygon_faces[f].size() == 3);

            auto f_vs = polygon_faces[f];
            std::sort(f_vs.begin(), f_vs.end());

            int64_t local_f_idx = -1;

            // decide which face it is

            if (f_vs == local_f0) {
                local_f_idx = 0;
            } else if (f_vs == local_f1) {
                local_f_idx = 1;
            } else if (f_vs == local_f2) {
                local_f_idx = 2;
            } else {
                local_f_idx = 3;
            }
            assert(local_f_idx >= 0);

            tet_face_on_input[local_f_idx] = polygon_faces_on_input[f];
        }

        tet_face_on_input_surface.push_back(tet_face_on_input);
    }
    timer.stop();
    wmtk::logger().info("took: {}", timer.getElapsedTimeInSec());

    wmtk::logger().info("removing unreferenced vertices...");
    timer.start();
    std::vector<bool> v_is_used_in_tet(v_coords.size(), false);
    for (const auto& t : tets_final) {
        for (const auto& v : t) {
            v_is_used_in_tet[v] = true;
        }
    }
    std::vector<int64_t> v_map(v_coords.size(), -1);
    std::vector<Vector3r> v_coords_final;

    for (int64_t i = 0; i < v_coords.size(); ++i) {
        if (v_is_used_in_tet[i]) {
            v_map[i] = v_coords_final.size();
            v_coords_final.emplace_back(v_coords[i]);
        }
    }
    for (auto& t : tets_final) {
        for (int i = 0; i < 4; ++i) {
            assert(v_map[t[i]] >= 0);
            t[i] = v_map[t[i]];
        }
    }
    timer.stop();
    wmtk::logger().info("took: {}", timer.getElapsedTimeInSec());


    // check tets
    // for (auto& t : tets_final) {
    //     if (wmtk_orient3d(
    //             v_coords_final[t[0]],
    //             v_coords_final[t[1]],
    //             v_coords_final[t[2]],
    //             v_coords_final[t[3]]) <= 0) {
    //         Eigen::Matrix<Rational, 3, 3> tmp;
    //         tmp.col(0) = v_coords_final[t[1]] - v_coords_final[t[0]];
    //         tmp.col(1) = v_coords_final[t[2]] - v_coords_final[t[0]];
    //         tmp.col(2) = v_coords_final[t[3]] - v_coords_final[t[0]];
    //         log_and_throw_error(
    //             "flipped tet=({},{},{},{}) crash vol={} orient={}",
    //             t[0],
    //             t[1],
    //             t[2],
    //             t[3],
    //             tmp.determinant().serialize(),
    //             wmtk_orient3d(
    //                 v_coords_final[t[0]],
    //                 v_coords_final[t[1]],
    //                 v_coords_final[t[2]],
    //                 v_coords_final[t[3]]));
    //     }
    // }

    wmtk::logger().info("convert to eigen...");
    // transfer v_coords_final to V matrix and tets_final to TV matrix
    RowVectors3r V_final_rational(v_coords_final.size(), 3);
    RowVectors4l TV_final(tets_final.size(), 4);

    for (int64_t i = 0; i < v_coords_final.size(); ++i) {
        V_final_rational.row(i) = v_coords_final[i];
    }

    for (int64_t i = 0; i < tets_final.size(); ++i) {
        assert(tets_final[i][0] >= 0);
        assert(tets_final[i][1] >= 0);
        assert(tets_final[i][2] >= 0);
        assert(tets_final[i][3] >= 0);
        assert(tets_final[i][0] < V_final_rational.rows());
        assert(tets_final[i][1] < V_final_rational.rows());
        assert(tets_final[i][2] < V_final_rational.rows());
        assert(tets_final[i][3] < V_final_rational.rows());

        assert(tets_final[i][0] != tets_final[i][1]);
        assert(tets_final[i][0] != tets_final[i][2]);
        assert(tets_final[i][0] != tets_final[i][3]);

        assert(tets_final[i][1] != tets_final[i][2]);
        assert(tets_final[i][1] != tets_final[i][3]);

        assert(tets_final[i][2] != tets_final[i][3]);

        TV_final.row(i) << tets_final[i][0], tets_final[i][1], tets_final[i][2], tets_final[i][3];
    }
    wmtk::logger().info("done");

    wmtk::logger().info("init tetmesh...");
    // initialize tetmesh
    std::shared_ptr<wmtk::TetMesh> m = std::make_shared<wmtk::TetMesh>();
    m->initialize(TV_final);
    // mesh_utils::set_matrix_attribute(V_final, "vertices", PrimitiveType::Vertex, *m);
    mesh_utils::set_matrix_attribute(V_final_rational, "vertices", PrimitiveType::Vertex, *m);

    wmtk::logger().info("init tetmesh finished.");

    if (!m->is_connectivity_valid()) {
        wmtk::logger().error("invalid tetmesh connectivity after insertion");
        throw std::runtime_error("invalid tetmesh connectivity by insertion");
    }
    return std::make_tuple(m, tet_face_on_input_surface);
}


} // namespace wmtk::utils
