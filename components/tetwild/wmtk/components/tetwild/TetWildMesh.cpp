
#include "TetWildMesh.h"

#include "wmtk/utils/Rational.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/concurrent_vector.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/fmt/bundled/format.h>
#include <igl/predicates/predicates.h>
#include <igl/winding_number.h>
#include <igl/write_triangle_mesh.h>
#include <igl/Timer.h>
#include <igl/orientable_patches.h>
#include <wmtk/utils/EnableWarnings.hpp>
#include <wmtk/utils/GeoUtils.h>
// clang-format on

//#include <paraviewo/HDF5VTUWriter.hpp>
#include <bitset>
#include <limits>
#include <paraviewo/VTUWriter.hpp>

#include "orig/Args.h"
#include "orig/MeshRefinement.h"
#include "orig/State.h"

namespace {
static int debug_print_counter = 0;
}

namespace wmtk::components::tetwild {


VertexAttributes::VertexAttributes(const Vector3r& p)
{
    m_pos = p;
    m_posf = to_double(p);
}

void TetWildMesh::mesh_improvement(int max_its)
{
    ////preprocessing
    // TODO: refactor to eliminate repeated partition.
    //

    // rounding
    // std::atomic_int cnt_round(0);
    // std::atomic_int cnt_valid(0);

    // auto vertices = get_vertices();
    // for (auto v : vertices) {
    //     // debug code
    //     if (v.is_valid(*this)) cnt_valid++;

    //     if (round(v)) cnt_round++;
    // }

    // wmtk::logger().info("cnt_round {}/{}", cnt_round, cnt_valid);

    compute_vertex_partition_morton();

    save_paraview(fmt::format("debug_{}", debug_print_counter++), false);

    wmtk::logger().info("========it pre========");
    local_operations({{0, 1, 0, 0}}, false);

    ////operation loops
    bool is_hit_min_edge_length = false;
    const int M = 2;
    int m = 0;
    double pre_max_energy = 0., pre_avg_energy = 0.;
    for (int it = 0; it < max_its; it++) {
        ///ops
        wmtk::logger().info("\n========it {}========", it);
        auto [max_energy, avg_energy] = local_operations({{1, 1, 5, 1}});

        ///energy check
        wmtk::logger().info("max energy {} stop {}", max_energy, m_params.stop_energy);
        if (max_energy < m_params.stop_energy) break;
        consolidate_mesh();

        // output_faces(
        //     m_params.output_path + "after_iter" + std::to_string(it) + ".obj",
        //     [](auto& f) { return f.m_is_surface_fs; });

        // output_mesh(m_params.output_path + "after_iter" + std::to_string(it) + ".msh");

        wmtk::logger().info("v {} t {}", vert_capacity(), tet_capacity());

        auto cnt_round = 0, cnt_verts = 0;
        TetMesh::for_each_vertex([&](auto& v) {
            if (m_vertex_attribute[v.vid(*this)].m_is_rounded) cnt_round++;
            cnt_verts++;
        });
        if (cnt_round < cnt_verts) {
            wmtk::logger().info("rounded {}/{}", cnt_round, cnt_verts);
        } else {
            wmtk::logger().info("All rounded!", cnt_round, cnt_verts);
        }

        ///sizing field
        if (it > 0 && pre_max_energy - max_energy < 5e-1 &&
            (pre_avg_energy - avg_energy) / avg_energy < 0.1) {
            m++;
            if (m == M) {
                wmtk::logger().info(">>>>adjust_sizing_field...");
                is_hit_min_edge_length = adjust_sizing_field_serial(max_energy);
                // is_hit_min_edge_length = adjust_sizing_field(max_energy);
                wmtk::logger().info(">>>>adjust_sizing_field finished...");
                m = 0;
            }
        } else {
            m = 0;
            pre_max_energy = max_energy;
            pre_avg_energy = avg_energy;
        }
        if (is_hit_min_edge_length) {
            // todo: maybe to do sth
        }
    }

    wmtk::logger().info("========it post========");
    local_operations({{0, 1, 0, 0}});
}

void TetWildMesh::mesh_improvement_legacy(int max_its)
{
    logger().set_level(spdlog::level::level_enum::debug);

    SampleEnvelope* env;
    if (SampleEnvelope* d = dynamic_cast<SampleEnvelope*>(&m_envelope); d != nullptr) {
        env = d;
    } else {
        log_and_throw_error("Legacy TetWild can only be used with sample envelope.");
    }
    SampleEnvelope* env_b;
    if (SampleEnvelope* d = dynamic_cast<SampleEnvelope*>(&m_envelope); d != nullptr) {
        env_b = d;
    } else {
        log_and_throw_error("Legacy TetWild can only be used with sample envelope.");
    }

    orig::Args args;
    args.initial_edge_len_rel = m_params.lr;
    args.initial_edge_len_abs = m_params.l;
    args.eps_rel = m_params.epsr;
    args.filter_energy_thres = m_params.stop_energy;
    args.max_num_passes = max_its;
    orig::State state(args, m_params.diag_l); // TODO check if params are correct
    {
        // check if mesh is closed
        state.is_mesh_closed = true;
        for (const Tuple& v : get_vertices()) {
            const size_t vid = v.vid(*this);
            if (m_vertex_attribute[vid].m_is_on_open_boundary) {
                state.is_mesh_closed = false;
                break;
            }
        }
    }

    orig::MeshRefinement legacy_tetwild(*env, *env_b, args, state);
    // add tets and vertices
    {
        // tets
        assert(legacy_tetwild.tets.empty());
        legacy_tetwild.tets.reserve(tet_capacity());
        for (size_t i = 0; i < tet_capacity(); ++i) {
            const auto vi = oriented_tet_vids(i);
            std::array<int, 4> v;
            for (size_t j = 0; j < 4; ++j) {
                v[j] = vi[j];
            }
            legacy_tetwild.tets.emplace_back(v);
        }

        // is_surface faces
        legacy_tetwild.is_surface_fs.resize(tet_capacity());
        for (size_t i = 0; i < legacy_tetwild.is_surface_fs.size(); ++i) {
            for (size_t j = 0; j < 4; ++j) {
                legacy_tetwild.is_surface_fs[i][j] = state.NOT_SURFACE;
            }
        }

        for (size_t i = 0; i < legacy_tetwild.tets.size(); ++i) {
            const auto& v = oriented_tet_vids(i);
            const auto [f0, fid0] = tuple_from_face({{v[1], v[2], v[3]}});
            const auto [f1, fid1] = tuple_from_face({{v[0], v[2], v[3]}});
            const auto [f2, fid2] = tuple_from_face({{v[0], v[1], v[3]}});
            const auto [f3, fid3] = tuple_from_face({{v[0], v[1], v[2]}});

            if (m_face_attribute[fid0].m_is_surface_fs) {
                legacy_tetwild.is_surface_fs[i][0] = 1;
            }
            if (m_face_attribute[fid1].m_is_surface_fs) {
                legacy_tetwild.is_surface_fs[i][1] = 1;
            }
            if (m_face_attribute[fid2].m_is_surface_fs) {
                legacy_tetwild.is_surface_fs[i][2] = 1;
            }
            if (m_face_attribute[fid3].m_is_surface_fs) {
                legacy_tetwild.is_surface_fs[i][3] = 1;
            }
        }

        // faces
        std::map<int, int> bbx_fids;
        bbx_fids[0] = -1;
        bbx_fids[1] = -2;
        bbx_fids[2] = -3;
        bbx_fids[3] = -4;
        bbx_fids[4] = -5;
        bbx_fids[5] = -6;
        // edges
        std::map<std::set<int>, int> bbx_eids;
        // 5
        // 0, 2, 1, 3
        // 4
        bbx_eids[std::set<int>{5, 0}] = -1;
        bbx_eids[std::set<int>{5, 2}] = -2;
        bbx_eids[std::set<int>{5, 1}] = -3;
        bbx_eids[std::set<int>{5, 3}] = -4;
        bbx_eids[std::set<int>{0, 2}] = -5;
        bbx_eids[std::set<int>{0, 3}] = -6;
        bbx_eids[std::set<int>{2, 1}] = -7;
        bbx_eids[std::set<int>{1, 3}] = -8;
        bbx_eids[std::set<int>{4, 0}] = -9;
        bbx_eids[std::set<int>{4, 2}] = -10;
        bbx_eids[std::set<int>{4, 1}] = -11;
        bbx_eids[std::set<int>{4, 3}] = -12;

        // vertices
        legacy_tetwild.tet_vertices.resize(vert_capacity());
        for (size_t i = 0; i < vert_capacity(); ++i) {
            const auto& VA = m_vertex_attribute[i];
            orig::TetVertex& v = legacy_tetwild.tet_vertices[i];
            v.pos = VA.m_pos;
            v.posf = VA.m_posf;
            v.is_on_bbox = !VA.on_bbox_faces.empty();
            if (v.is_on_bbox) {
                /**
                 * This is a bit ugly but we need to assign all bbox faces and edges a unique ID
                 */
                for (const int fid : VA.on_bbox_faces) {
                    v.on_face.insert(bbx_fids[fid]);
                }

                if (VA.on_bbox_faces.size() == 2) {
                    std::set<int> bbx(VA.on_bbox_faces.begin(), VA.on_bbox_faces.end());
                    v.on_edge.insert(bbx_eids[bbx]);
                } else if (VA.on_bbox_faces.size() == 3) {
                    // bbox corner
                    for (size_t j = 0; j < 3; ++j) {
                        std::set<int> bbx{VA.on_bbox_faces[j], VA.on_bbox_faces[(j + 1) % 3]};
                        v.on_edge.insert(bbx_eids[bbx]);
                    }
                    v.on_fixed_vertex = true; // cannot move corner
                } else if (VA.on_bbox_faces.size() > 3) {
                    log_and_throw_error("Vertex should not be on more than 3 bbox faces");
                }
            }
            v.is_on_boundary = VA.m_is_on_open_boundary;
            v.is_on_surface = VA.m_is_on_surface;
            v.is_rounded = VA.m_is_rounded;
            for (const size_t tid : get_one_ring_tids_for_vertex(i)) {
                v.conn_tets.insert(tid);
            }
        }
    }

    legacy_tetwild.prepareData();
    legacy_tetwild.refine(); // the actual tetwild

    // write back to our format
    logger().info("Write back to WMTK format");
    {
        const auto& tets = legacy_tetwild.tets;
        size_t tet_count = std::count(
            legacy_tetwild.t_is_removed.begin(),
            legacy_tetwild.t_is_removed.end(),
            false);

        MatrixXi T;
        T.resize(tet_count, 4);
        tet_count = 0;
        for (size_t i = 0; i < tets.size(); ++i) {
            if (legacy_tetwild.t_is_removed[i]) {
                continue;
            }
            for (size_t j = 0; j < 4; ++j) {
                T(tet_count, j) = tets[i][j];
            }
            ++tet_count;
        }

        init(T);

        const auto& verts = legacy_tetwild.tet_vertices;
        assert(verts.size() >= vert_capacity());
        for (size_t i = 0; i < vert_capacity(); ++i) {
            auto& VA = m_vertex_attribute[i];
            const orig::TetVertex& v = verts[i];
            VA.m_is_rounded = v.is_rounded;
            if (v.is_rounded) {
                VA.m_pos = to_rational(v.posf);
                VA.m_posf = v.posf;
            } else {
                VA.m_pos = v.pos;
                VA.m_posf = to_double(v.pos);
            }
            VA.m_sizing_scalar = v.adaptive_scale;
            VA.m_is_on_surface = v.is_on_surface;
            VA.m_is_on_open_boundary = v.is_on_boundary;
            // logger().info("DEBUG on_bbox");
            if (v.is_on_bbox) {
                VA.on_bbox_faces.clear();
                for (const int id : v.on_face) {
                    VA.on_bbox_faces.emplace_back(1 - id);
                }
                std::sort(VA.on_bbox_faces.begin(), VA.on_bbox_faces.end());
            }
        }
        for (size_t i = vert_capacity(); i < verts.size(); ++i) {
            if (!legacy_tetwild.v_is_removed[i]) {
                logger().error("Vertex {} is not removed", i);
            }
        }

        tet_count = 0;
        for (size_t i = 0; i < tets.size(); ++i) {
            if (legacy_tetwild.t_is_removed[i]) {
                continue;
            }
            const auto& v = oriented_tet_vids(tet_count);
            const auto [f0, fid0] = tuple_from_face({{v[1], v[2], v[3]}});
            const auto [f1, fid1] = tuple_from_face({{v[0], v[2], v[3]}});
            const auto [f2, fid2] = tuple_from_face({{v[0], v[1], v[3]}});
            const auto [f3, fid3] = tuple_from_face({{v[0], v[1], v[2]}});

            if (legacy_tetwild.is_surface_fs[i][0] == 1) {
                m_face_attribute[fid0].m_is_surface_fs = true;
            }
            if (legacy_tetwild.is_surface_fs[i][1] == 1) {
                m_face_attribute[fid1].m_is_surface_fs = true;
            }
            if (legacy_tetwild.is_surface_fs[i][2] == 1) {
                m_face_attribute[fid2].m_is_surface_fs = true;
            }
            if (legacy_tetwild.is_surface_fs[i][3] == 1) {
                m_face_attribute[fid3].m_is_surface_fs = true;
            }
            ++tet_count;
        }

        for (const Tuple& t : get_tets()) {
            // This could be also transferred from legacy_tetwild but I wanted to do that here to
            // ensure the energy is computed in the same way.
            const double e = get_quality(t);
            m_tet_attribute[t.tid(*this)].m_quality = e;
        }
    }
    logger().info("Finish legacy mesh refinement.");
}

std::tuple<double, double> TetWildMesh::local_operations(
    const std::array<int, 4>& ops,
    bool collapse_limit_length)
{
    igl::Timer timer;

    std::tuple<double, double> energy;

    auto sanity_checks = [this]() {
        if (!m_params.perform_sanity_checks) {
            return;
        }
        logger().info("Perform sanity checks...");
        const auto faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });
        for (const auto& verts : faces) {
            const auto p0 = m_vertex_attribute[verts[0]].m_posf;
            const auto p1 = m_vertex_attribute[verts[1]].m_posf;
            const auto p2 = m_vertex_attribute[verts[2]].m_posf;
            if (m_envelope.is_outside({{p0, p1, p2}})) {
                logger().error("Face {} is outside!", verts);
            }
        }

        // check for inverted tets
        for (const Tuple& t : get_tets()) {
            if (!is_inverted(t)) {
                continue;
            }
            const auto vs = oriented_tet_vids(t);
            logger().error("Tet {} is inverted! Vertices = {}", t.tid(*this), vs);
            // for (const size_t v : vs) {
            //     logger().error(
            //         "v{}, rounded = {}, on surface = {}, on open boundary = {}",
            //         v,
            //         m_vertex_attribute[v].m_is_rounded,
            //         m_vertex_attribute[v].m_is_on_surface,
            //         m_vertex_attribute[v].m_is_on_open_boundary);
            // }
        }

        // check boundary envelope
        // note that edges can end up outside during collapse and that is desired behavior to
        // collapse thin ribbons
        {
            const auto fs = get_faces();
            const auto es = get_edges();
            std::vector<int> edge_on_open_boundary(6 * tet_capacity(), 0);

            for (const Tuple& f : fs) {
                auto fid = f.fid(*this);
                if (!m_face_attribute[fid].m_is_surface_fs) {
                    continue;
                }
                size_t eid1 = f.eid(*this);
                size_t eid2 = f.switch_edge(*this).eid(*this);
                size_t eid3 = f.switch_vertex(*this).switch_edge(*this).eid(*this);

                edge_on_open_boundary[eid1]++;
                edge_on_open_boundary[eid2]++;
                edge_on_open_boundary[eid3]++;
            }

            for (const Tuple& e : es) {
                if (edge_on_open_boundary[e.eid(*this)] != 1) {
                    continue;
                }
                if (!is_open_boundary_edge(e)) {
                    size_t v1 = e.vid(*this);
                    size_t v2 = e.switch_vertex(*this).vid(*this);
                    if (!m_vertex_attribute[v1].m_is_on_open_boundary ||
                        !m_vertex_attribute[v2].m_is_on_open_boundary) {
                        continue;
                    }
                    logger().warn("Boundary edge ({},{}) is outside the envelope.", v1, v2);
                    // logger().error(
                    //     "v{}, on surface = {}, on open boundary = {}",
                    //     v1,
                    //     m_vertex_attribute[v1].m_is_on_surface,
                    //     m_vertex_attribute[v1].m_is_on_open_boundary);
                    // logger().error(
                    //     "v{}, on surface = {}, on open boundary = {}",
                    //     v2,
                    //     m_vertex_attribute[v2].m_is_on_surface,
                    //     m_vertex_attribute[v2].m_is_on_open_boundary);
                }
            }
        }
        logger().info("Sanity checks done.");
    };

    sanity_checks();

    for (int i = 0; i < ops.size(); i++) {
        timer.start();
        if (i == 0) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==splitting {}==", n);
                split_all_edges();
                wmtk::logger().info(
                    "#vertices {}, #tets {} after split",
                    get_vertices().size(),
                    get_tets().size());
                // auto faces = get_faces();
                // for (auto f : faces) {
                //     auto x = f.fid(*this);
                // }
                // if (!check_vertex_param_type()) {
                //     std::cout << "missing param!!!!!!!!" << std::endl;
                //     output_faces("bug_surface_miss_param_after_split.obj", [](auto& f) {
                //         return f.m_is_surface_fs;
                //     });
                //     // exit(0);
                // }
            }
            // save_paraview(fmt::format("debug_{}", debug_print_counter++), false);
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("split max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 1) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==collapsing {}==", n);
                collapse_all_edges(collapse_limit_length);
                wmtk::logger().info(
                    "#vertices {}, #tets {} after collapse",
                    get_vertices().size(),
                    get_tets().size());
                // auto faces = get_faces();
                // for (auto f : faces) {
                //     auto x = f.fid(*this);
                // }
                // if (!check_vertex_param_type()) {
                //     std::cout << "missing param!!!!!!!!" << std::endl;
                //     output_faces("buf_surface_miss_param_after_collpase.obj", [](auto& f) {
                //         return f.m_is_surface_fs;
                //     });
                //     // exit(0);
                // }
            }
            // save_paraview(fmt::format("debug_{}", debug_print_counter++), false);
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("collapse max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 2) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==swapping {}==", n);
                int cnt_success = 0;
                cnt_success += swap_all_edges_56();
                cnt_success += swap_all_edges_44();
                cnt_success += swap_all_edges();
                cnt_success += swap_all_faces();
                if (cnt_success == 0) {
                    break;
                }
            }
            // save_paraview(fmt::format("debug_{}", debug_print_counter++), false);
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("swap max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 3) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==smoothing {}==", n);
                smooth_all_vertices();
            }
            // save_paraview(fmt::format("debug_{}", debug_print_counter++), false);
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("smooth max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        }
        // output_faces(fmt::format("out-op{}.obj", i), [](auto& f) { return f.m_is_surface_fs; });
    }
    // save_paraview(fmt::format("debug_{}", debug_print_counter++), false);
    energy = get_max_avg_energy();
    wmtk::logger().info("max energy = {:.6}", std::get<0>(energy));
    wmtk::logger().info("avg energy = {:.6}", std::get<1>(energy));
    wmtk::logger().info("time = {}", timer.getElapsedTime());


    return energy;
}

bool TetWildMesh::adjust_sizing_field_serial(double max_energy)
{
    wmtk::logger().info("#vertices {}, #tets {}", vert_capacity(), tet_capacity());

    const double stop_filter_energy = m_params.stop_energy * 0.8;
    double filter_energy = std::max(max_energy / 100, stop_filter_energy);
    filter_energy = std::min(filter_energy, 100.);

    const auto recover_scalar = 1.5;
    const auto refine_scalar = 0.5;
    const auto min_refine_scalar = m_params.l_min / m_params.l;

    // outputs scale_multipliers
    std::vector<double> scale_multipliers(vert_capacity(), recover_scalar);

    std::vector<Vector3d> pts;
    std::queue<size_t> v_queue;

    for (int i = 0; i < tet_capacity(); i++) {
        auto t = tuple_from_tet(i);
        if (!t.is_valid(*this)) continue;
        auto tid = t.tid(*this);
        if (std::cbrt(m_tet_attribute[tid].m_quality) < filter_energy) continue;
        auto vs = oriented_tet_vids(t);
        Vector3d c(0, 0, 0);
        for (int j = 0; j < 4; j++) {
            c += (m_vertex_attribute[vs[j]].m_posf);
            v_queue.emplace(vs[j]);
            // std::cout << vs[j] << " ";
        }
        pts.emplace_back(c / 4);
    }

    // std::cout << std::endl;


    wmtk::logger().info("filter energy {} Low Quality Tets {}", filter_energy, pts.size());

    // debug code
    // std::queue<size_t> v_queue_serial;
    // for (tbb::concurrent_queue<size_t>::const_iterator i(v_queue.unsafe_begin());
    //      i != v_queue.unsafe_end();
    //      ++i) {
    //     // std::cout << *i << " ";
    //     v_queue_serial.push(*i);
    // }
    // std::cout << std::endl;

    const double R = m_params.l * 1.8;

    int sum = 0;
    int adjcnt = 0;

    std::vector<bool> visited(vert_capacity(), false);

    KNN knn(pts);

    std::vector<size_t> cache_one_ring;
    // size_t vid;

    while (!v_queue.empty()) {
        // std::cout << vid << " ";
        sum++;
        size_t vid = v_queue.front();
        // std::cout << vid << " ";
        v_queue.pop();
        if (visited[vid]) continue;
        visited[vid] = true;
        adjcnt++;

        auto& pos_v = m_vertex_attribute[vid].m_posf;
        double sq_dist = 0.;
        uint32_t idx;
        knn.nearest_neighbor(pos_v, idx, sq_dist);
        const double dist = std::sqrt(sq_dist);

        if (dist > R) { // outside R-ball, unmark.
            continue;
        }

        scale_multipliers[vid] = std::min(
            scale_multipliers[vid],
            dist / R * (1 - refine_scalar) + refine_scalar); // linear interpolate

        auto vids = get_one_ring_vids_for_vertex_adj(vid, cache_one_ring);
        for (size_t n_vid : vids) {
            if (visited[n_vid]) continue;
            v_queue.push(n_vid);
        }
    }

    // std::cout << std::endl;

    std::cout << sum << " " << adjcnt << std::endl;

    std::atomic_bool is_hit_min_edge_length = false;

    for (int i = 0; i < vert_capacity(); i++) {
        auto v = tuple_from_vertex(i);
        if (!v.is_valid(*this)) continue;
        auto vid = v.vid(*this);
        // std::cout << vid << " ";
        auto& v_attr = m_vertex_attribute[vid];

        auto new_scale = v_attr.m_sizing_scalar * scale_multipliers[vid];
        if (new_scale > 1)
            v_attr.m_sizing_scalar = 1;
        else if (new_scale < min_refine_scalar) {
            is_hit_min_edge_length = true;
            v_attr.m_sizing_scalar = min_refine_scalar;
        } else
            v_attr.m_sizing_scalar = new_scale;
    }
    // std::cout << std::endl;

    return is_hit_min_edge_length.load();
}

void bfs_orient(const Eigen::MatrixXi& F, Eigen::MatrixXi& FF, Eigen::VectorXi& C)
{
    Eigen::SparseMatrix<int> A;
    igl::orientable_patches(F, C, A);

    // number of faces
    const int m = F.rows();
    // number of patches
    const int num_cc = C.maxCoeff() + 1;
    Eigen::VectorXi seen = Eigen::VectorXi::Zero(m);

    // Edge sets
    const int ES[3][2] = {{1, 2}, {2, 0}, {0, 1}};

    if (((void*)&FF) != ((void*)&F)) FF = F;

    // loop over patches
    for (int c = 0; c < num_cc; c++) {
        std::queue<int> Q;
        // find first member of patch c
        int cnt = 0;
        for (int f = 0; f < FF.rows(); f++) {
            if (C(f) == c) {
                if (cnt == 0) Q.push(f);
                cnt++;
                //                break;
            }
        }
        if (cnt < 5) continue;

        int cnt_inverted = 0;
        assert(!Q.empty());
        while (!Q.empty()) {
            const int f = Q.front();
            Q.pop();
            if (seen(f) > 0) continue;

            seen(f)++;
            // loop over neighbors of f
            for (Eigen::SparseMatrix<int>::InnerIterator it(A, f); it; ++it) {
                // might be some lingering zeros, and skip self-adjacency
                if (it.value() != 0 && it.row() != f) {
                    const int n = it.row();
                    assert(n != f);
                    // loop over edges of f
                    for (int efi = 0; efi < 3; efi++) {
                        // efi'th edge of face f
                        Eigen::Vector2i ef(FF(f, ES[efi][0]), FF(f, ES[efi][1]));
                        // loop over edges of n
                        for (int eni = 0; eni < 3; eni++) {
                            // eni'th edge of face n
                            Eigen::Vector2i en(FF(n, ES[eni][0]), FF(n, ES[eni][1]));
                            // Match (half-edges go same direction)
                            if (ef(0) == en(0) && ef(1) == en(1)) {
                                // flip face n
                                FF.row(n) = FF.row(n).reverse().eval();
                                cnt_inverted++;
                            }
                        }
                    }
                    // add neighbor to queue
                    Q.push(n);
                }
            }
        }
        if (cnt_inverted < cnt / 2) continue;

        for (int f = 0; f < FF.rows(); f++) {
            if (C(f) == c) FF.row(f) = FF.row(f).reverse().eval();
        }
    }
}

void TetWildMesh::compute_winding_number(
    const std::vector<Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    if (!vertices.empty()) {
        V.resize(vertices.size(), 3);
        F.resize(faces.size(), 3);

        for (int i = 0; i < V.rows(); i++) {
            V.row(i) = vertices[i];
        }
        for (int i = 0; i < F.rows(); i++) {
            for (auto j = 0; j < 3; j++) F(i, j) = faces[i][j];
        }
    } else { // use track to filter
        auto outface = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });
        V = Eigen::MatrixXd::Zero(vert_capacity(), 3);
        for (auto v : get_vertices()) {
            auto vid = v.vid(*this);
            V.row(vid) = m_vertex_attribute[vid].m_posf;
        }
        F.resize(outface.size(), 3);
        for (auto i = 0; i < outface.size(); i++) {
            F.row(i) << outface[i][0], outface[i][1], outface[i][2];
        }
        // wmtk::logger().info("Output face size {}", outface.size());
        auto F0 = F;
        Eigen::VectorXi C;
        bfs_orient(F0, F, C);
        // wmtk::logger().info("BFS orient {}", F.rows());
    }

    const auto& tets = get_tets();
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(tets.size(), 3);
    for (size_t i = 0; i < tets.size(); i++) {
        auto vs = oriented_tet_vertices(tets[i]);
        for (auto& v : vs) C.row(i) += m_vertex_attribute[v.vid(*this)].m_posf;
        C.row(i) /= 4;
    }

    Eigen::VectorXd W;
    igl::winding_number(V, F, C, W);

    if (W.maxCoeff() <= 0.5) {
        // all removed, let's invert.
        wmtk::logger().info("Correcting winding number");
        for (auto i = 0; i < F.rows(); i++) {
            auto temp = F(i, 0);
            F(i, 0) = F(i, 1);
            F(i, 1) = temp;
        }
        igl::winding_number(V, F, C, W);
    }

    if (W.maxCoeff() <= 0.5) {
        wmtk::logger().critical("Still Inverting..., Empty Output");
        return;
    }

    // store winding number in mesh
    if (vertices.empty()) {
        // from tracked surface
        for (int i = 0; i < tets.size(); ++i) {
            const size_t tid = tets[i].tid(*this);
            m_tet_attribute[tid].m_winding_number_tracked = W(i);
        }
    } else {
        // from input surface
        for (int i = 0; i < tets.size(); ++i) {
            const size_t tid = tets[i].tid(*this);
            m_tet_attribute[tid].m_winding_number_input = W(i);
        }
    }
}

void TetWildMesh::filter_with_input_surface_winding_number()
{
    std::vector<size_t> rm_tids;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (m_tet_attribute[tid].m_winding_number_input <= 0.5) {
            rm_tids.emplace_back(tid);
        }
    }

    remove_tets_by_ids(rm_tids);
}

void TetWildMesh::filter_with_tracked_surface_winding_number()
{
    std::vector<size_t> rm_tids;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (m_tet_attribute[tid].m_winding_number_tracked <= 0.5) {
            rm_tids.emplace_back(tid);
        }
    }

    remove_tets_by_ids(rm_tids);
}

void TetWildMesh::filter_with_flood_fill()
{
    std::map<int, size_t> id_counter;

    // find ID that appears the most on the boundary
    for (const Tuple& t : get_faces()) {
        if (t.switch_tetrahedron(*this)) {
            // face is interior
            continue;
        }
        // face is boundary
        const int id = m_tet_attribute[t.tid(*this)].part_id;

        if (id_counter.count(id) == 0) {
            id_counter[id] = 1;
        } else {
            id_counter[id]++;
        }
    }

    if (id_counter.size() != 1) {
        logger().warn(
            "There were {} flood fill IDs found at the boundary. Using the one with most "
            "occurances.",
            id_counter.size());
    }

    int best_id = id_counter.begin()->first;
    size_t best_count = id_counter.begin()->second;
    for (const auto& [id, count] : id_counter) {
        if (count > best_count) {
            best_id = id;
            best_count = count;
        }
    }

    logger().info("Filter with flood fill ID {}", best_id);

    std::vector<size_t> rm_tids;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (m_tet_attribute[tid].part_id == best_id) {
            rm_tids.emplace_back(tid);
        }
    }

    remove_tets_by_ids(rm_tids);
}

/////////////////////////////////////////////////////////////////////
void TetWildMesh::output_faces(std::string file, std::function<bool(const FaceAttributes&)> cond)
{
    auto outface = get_faces_by_condition(cond);
    Eigen::MatrixXd matV = Eigen::MatrixXd::Zero(vert_capacity(), 3);
    for (const auto& v : get_vertices()) {
        auto vid = v.vid(*this);
        matV.row(vid) = m_vertex_attribute[vid].m_posf;
    }
    Eigen::MatrixXi matF(outface.size(), 3);
    for (auto i = 0; i < outface.size(); i++) {
        matF.row(i) << outface[i][0], outface[i][1], outface[i][2];
    }
    wmtk::logger().info("Output face size {}", outface.size());
    igl::write_triangle_mesh(file, matV, matF);
}


void TetWildMesh::output_mesh(std::string file)
{
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_tet_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        return m_vertex_attribute[i].m_posf;
    });

    const auto& tets = get_tets();
    msh.add_tets(tets.size(), [&](size_t k) {
        auto i = tets[k].tid(*this);
        auto vs = oriented_tet_vertices(tets[k]);
        std::array<size_t, 4> data;
        for (int j = 0; j < 4; j++) {
            data[j] = vs[j].vid(*this);
            assert(data[j] < vtx.size());
        }
        return data;
    });

    msh.add_tet_vertex_attribute<1>("tv index", [&](size_t i) {
        return m_vertex_attribute[i].m_sizing_scalar;
    });
    msh.add_tet_attribute<1>("t energy", [&](size_t i) {
        return std::cbrt(m_tet_attribute[i].m_quality);
    });
    msh.add_tet_attribute<1>("winding_number_input", [&](size_t i) {
        return std::cbrt(m_tet_attribute[i].m_winding_number_input);
    });
    msh.add_tet_attribute<1>("winding_number_tracked", [&](size_t i) {
        return std::cbrt(m_tet_attribute[i].m_winding_number_tracked);
    });
    msh.add_tet_attribute<1>("part", [&](size_t i) {
        return std::cbrt(m_tet_attribute[i].part_id);
    });

    msh.save(file, true);
}


double TetWildMesh::get_length2(const wmtk::TetMesh::Tuple& l) const
{
    auto& m = *this;
    auto& v1 = l;
    auto v2 = l.switch_vertex(m);
    double length =
        (m.m_vertex_attribute[v1.vid(m)].m_posf - m.m_vertex_attribute[v2.vid(m)].m_posf)
            .squaredNorm();
    return length;
}

std::tuple<double, double> TetWildMesh::get_max_avg_energy()
{
    double max_energy = -1.;
    double avg_energy = 0.;
    auto cnt = 0;
    // TetMesh::for_each_tetra([&](auto& t) {
    //     auto q = m_tet_attribute[t.tid(*this)].m_quality;
    //     max_energy = std::max(max_energy, q);
    //     avg_energy += std::cbrt(q);
    //     cnt++;
    // });
    // std::ofstream large_tet("large_energy_tet.obj");

    for (int i = 0; i < tet_capacity(); i++) {
        auto tup = tuple_from_tet(i);
        if (!tup.is_valid(*this)) continue;
        // auto vs = oriented_tet_vertices(tup);

        auto q = m_tet_attribute[tup.tid(*this)].m_quality;
        max_energy = std::max(max_energy, q);
        // if (q > 1e6) {
        //     for (auto v : vs) {
        //         large_tet << "v " << m_vertex_attribute[v.vid(*this)].m_posf[0] << " "
        //                   << m_vertex_attribute[v.vid(*this)].m_posf[1] << " "
        //                   << m_vertex_attribute[v.vid(*this)].m_posf[2] << std::endl;
        //     }
        // }
        avg_energy += std::cbrt(q);
        cnt++;
    }

    avg_energy /= cnt;

    return std::make_tuple(std::cbrt(max_energy), avg_energy);
}


bool TetWildMesh::is_inverted_f(const Tuple& loc) const
{
    auto vs = oriented_tet_vertices(loc);

    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(
        m_vertex_attribute[vs[0].vid(*this)].m_posf,
        m_vertex_attribute[vs[1].vid(*this)].m_posf,
        m_vertex_attribute[vs[2].vid(*this)].m_posf,
        m_vertex_attribute[vs[3].vid(*this)].m_posf);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
        return false;
    return true;
}

bool TetWildMesh::is_inverted(const std::array<size_t, 4>& vs) const
{
    // Return a positive value if the point pd lies below the
    // plane passing through pa, pb, and pc; "below" is defined so
    // that pa, pb, and pc appear in counterclockwise order when
    // viewed from above the plane.

    if (m_vertex_attribute[vs[0]].m_is_rounded && m_vertex_attribute[vs[1]].m_is_rounded &&
        m_vertex_attribute[vs[2]].m_is_rounded && m_vertex_attribute[vs[3]].m_is_rounded) {
        igl::predicates::exactinit();
        auto res = igl::predicates::orient3d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf,
            m_vertex_attribute[vs[3]].m_posf);
        int result;
        if (res == igl::predicates::Orientation::POSITIVE)
            result = 1;
        else if (res == igl::predicates::Orientation::NEGATIVE)
            result = -1;
        else
            result = 0;

        if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
            return false;
        return true;
    } else {
        Vector3r n =
            ((m_vertex_attribute[vs[1]].m_pos) - m_vertex_attribute[vs[0]].m_pos)
                .cross((m_vertex_attribute[vs[2]].m_pos) - m_vertex_attribute[vs[0]].m_pos);
        Vector3r d = (m_vertex_attribute[vs[3]].m_pos) - m_vertex_attribute[vs[0]].m_pos;
        auto res = n.dot(d);
        if (res > 0) // predicates returns pos value: non-inverted
            return false;
        else
            return true;
    }
}

bool TetWildMesh::is_inverted(const Tuple& loc) const
{
    auto vs = oriented_tet_vids(loc);
    return is_inverted(vs);
}

bool TetWildMesh::round(const Tuple& v)
{
    size_t i = v.vid(*this);
    if (m_vertex_attribute[i].m_is_rounded) return true;

    auto old_pos = m_vertex_attribute[i].m_pos;
    m_vertex_attribute[i].m_pos << m_vertex_attribute[i].m_posf[0], m_vertex_attribute[i].m_posf[1],
        m_vertex_attribute[i].m_posf[2];
    auto conn_tets = get_one_ring_tets_for_vertex(v);
    m_vertex_attribute[i].m_is_rounded = true;
    for (auto& tet : conn_tets) {
        if (is_inverted(tet)) {
            m_vertex_attribute[i].m_is_rounded = false;
            m_vertex_attribute[i].m_pos = old_pos;
            return false;
        }
    }

    return true;
}

double TetWildMesh::get_quality(const std::array<size_t, 4>& its) const
{
    std::array<Vector3d, 4> ps;
    auto use_rational = false;
    for (auto k = 0; k < 4; k++) {
        ps[k] = m_vertex_attribute[its[k]].m_posf;
        if (!m_vertex_attribute[its[k]].m_is_rounded) {
            use_rational = true;
            break;
        }
    }
    auto energy = -1.;
    if (!use_rational) {
        std::array<double, 12> T;
        for (auto k = 0; k < 4; k++)
            for (auto j = 0; j < 3; j++) T[k * 3 + j] = ps[k][j];

        energy = wmtk::AMIPS_energy_stable_p3<wmtk::Rational>(T);
    } else {
        std::array<wmtk::Rational, 12> T;
        for (auto k = 0; k < 4; k++)
            for (auto j = 0; j < 3; j++) T[k * 3 + j] = m_vertex_attribute[its[k]].m_pos[j];
        energy = wmtk::AMIPS_energy_rational_p3<wmtk::Rational>(T);
    }
    if (std::isinf(energy) || std::isnan(energy) || energy < 27 - 1e-3) return MAX_ENERGY;
    return energy;
}

double TetWildMesh::get_quality(const Tuple& loc) const
{
    auto its = oriented_tet_vids(loc);
    return get_quality(its);
}


bool TetWildMesh::invariants(const std::vector<Tuple>& tets)
{
    return true;
}

std::vector<std::array<size_t, 3>> TetWildMesh::get_faces_by_condition(
    std::function<bool(const FaceAttributes&)> cond)
{
    auto res = std::vector<std::array<size_t, 3>>();
    for (auto f : get_faces()) {
        auto fid = f.fid(*this);
        if (cond(m_face_attribute[fid])) {
            auto tid = fid / 4, lid = fid % 4;
            auto verts = get_face_vertices(f);
            res.emplace_back(std::array<size_t, 3>{
                {verts[0].vid(*this), verts[1].vid(*this), verts[2].vid(*this)}});
        }
    }
    return res;
}

bool TetWildMesh::is_edge_on_surface(const Tuple& loc)
{
    size_t v1_id = loc.vid(*this);
    auto loc1 = loc.switch_vertex(*this);
    size_t v2_id = loc1.vid(*this);
    if (!m_vertex_attribute[v1_id].m_is_on_surface || !m_vertex_attribute[v2_id].m_is_on_surface)
        return false;

    auto tets = get_incident_tets_for_edge(loc);
    std::vector<size_t> n_vids;
    for (auto& t : tets) {
        auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            if (vs[j].vid(*this) != v1_id && vs[j].vid(*this) != v2_id)
                n_vids.push_back(vs[j].vid(*this));
        }
    }
    wmtk::vector_unique(n_vids);

    for (size_t vid : n_vids) {
        auto [_, fid] = tuple_from_face({{v1_id, v2_id, vid}});
        if (m_face_attribute[fid].m_is_surface_fs) return true;
    }

    return false;
}


bool TetWildMesh::is_edge_on_bbox(const Tuple& loc)
{
    size_t v1_id = loc.vid(*this);
    auto loc1 = loc.switch_vertex(*this);
    size_t v2_id = loc1.vid(*this);
    if (m_vertex_attribute[v1_id].on_bbox_faces.empty() ||
        m_vertex_attribute[v2_id].on_bbox_faces.empty())
        return false;

    auto tets = get_incident_tets_for_edge(loc);
    std::vector<size_t> n_vids;
    for (auto& t : tets) {
        auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            if (vs[j].vid(*this) != v1_id && vs[j].vid(*this) != v2_id)
                n_vids.push_back(vs[j].vid(*this));
        }
    }
    wmtk::vector_unique(n_vids);

    for (size_t vid : n_vids) {
        auto [_, fid] = tuple_from_face({{v1_id, v2_id, vid}});
        if (m_face_attribute[fid].m_is_bbox_fs >= 0) return true;
    }

    return false;
}

bool TetWildMesh::check_attributes()
{
    for (auto& f : get_faces()) {
        auto fid = f.fid(*this);
        auto vs = get_face_vertices(f);

        if (m_face_attribute[fid].m_is_surface_fs) {
            if (!(m_vertex_attribute[vs[0].vid(*this)].m_is_on_surface &&
                  m_vertex_attribute[vs[1].vid(*this)].m_is_on_surface &&
                  m_vertex_attribute[vs[2].vid(*this)].m_is_on_surface)) {
                wmtk::logger().critical("surface track wrong");
                return false;
            }
            bool is_out = m_envelope.is_outside(
                {{m_vertex_attribute[vs[0].vid(*this)].m_posf,
                  m_vertex_attribute[vs[1].vid(*this)].m_posf,
                  m_vertex_attribute[vs[2].vid(*this)].m_posf}});
            if (is_out) {
                wmtk::logger().critical(
                    "is_out f {} {} {}",
                    vs[0].vid(*this),
                    vs[1].vid(*this),
                    vs[2].vid(*this));
                return false;
            }
        }
        if (m_face_attribute[fid].m_is_bbox_fs >= 0) {
            if (!(!m_vertex_attribute[vs[0].vid(*this)].on_bbox_faces.empty() &&
                  !m_vertex_attribute[vs[1].vid(*this)].on_bbox_faces.empty() &&
                  !m_vertex_attribute[vs[2].vid(*this)].on_bbox_faces.empty())) {
                wmtk::logger().critical("bbox track wrong {}", fid);
                return false;
            }
        }
    }

    const auto& vs = get_vertices();
    for (const auto& v : vs) {
        size_t i = v.vid(*this);
        if (m_vertex_attribute[i].m_is_on_surface) {
            bool is_out = m_envelope.is_outside(m_vertex_attribute[i].m_posf);
            if (is_out) {
                wmtk::logger().critical("is_out v");
                return false;
            }
        }

        // check rounding
        if (m_vertex_attribute[i].m_is_rounded) {
            if (m_vertex_attribute[i].m_pos[0] != m_vertex_attribute[i].m_posf[0] ||
                m_vertex_attribute[i].m_pos[1] != m_vertex_attribute[i].m_posf[1] ||
                m_vertex_attribute[i].m_pos[2] != m_vertex_attribute[i].m_posf[2]) {
                wmtk::logger().critical("rounding error {} rounded", i);
                return false;
            }
        } else {
            Vector3d p = to_double(m_vertex_attribute[i].m_pos);
            if (p != m_vertex_attribute[i].m_posf) {
                wmtk::logger().critical("rounding error {} unrounded", i);
                return false;
            }
        }
    }

    // check quality
    const auto& tets = get_tets();
    for (const auto& t : tets) {
        size_t i = t.tid(*this);
        double q = get_quality(t);
        if (q != m_tet_attribute[i].m_quality) {
            wmtk::logger().critical(
                "q!=m_tet_attribute[i].m_quality {} {}",
                q,
                m_tet_attribute[i].m_quality);
            return false;
        }
    }
    return true;
}

long long TetWildMesh::checksum_vidx()
{
    long long checksum = 0;
    auto vs = get_vertices();
    for (int i = 0; i < vs.size(); i++) {
        if (vs[i].is_valid(*this)) checksum += i;
    }
    return checksum;
}

long long TetWildMesh::checksum_tidx()
{
    long long checksum = 0;
    auto ts = get_tets();
    for (int i = 0; i < ts.size(); i++) {
        if (ts[i].is_valid(*this)) checksum += i;
    }
    return checksum;
}

// util functions for union find
int find_uf(int v, std::vector<int>& parent)
{
    int root = v;
    while (parent[root] != root) {
        root = parent[root];
    }
    // path compression optimization
    while (parent[v] != root) {
        int next = parent[v];
        parent[v] = root;
        v = next;
    }
    return root;
}

void union_uf(int u, int v, std::vector<int>& parent)
{
    int root_u = find_uf(u, parent);
    int root_v = find_uf(v, parent);
    if (root_u != root_v) {
        parent[root_u] = root_v;
    }
}

int TetWildMesh::count_vertex_links(const Tuple& v)
{
    // get one ring faces on surface
    auto one_ring_tets = get_one_ring_tets_for_vertex(v);
    std::vector<Tuple> surface_fs;
    for (auto t : one_ring_tets) {
        Tuple f1 = t;
        Tuple f2 = t.switch_face(*this);
        Tuple f3 = t.switch_edge(*this).switch_face(*this);
        Tuple f4 = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);
        // if (m_face_attribute[f1.fid(*this)].m_is_surface_fs) surface_fs.push_back(f1);
        // if (m_face_attribute[f2.fid(*this)].m_is_surface_fs) surface_fs.push_back(f2);
        // if (m_face_attribute[f3.fid(*this)].m_is_surface_fs) surface_fs.push_back(f3);
        // if (m_face_attribute[f4.fid(*this)].m_is_surface_fs) surface_fs.push_back(f4);
        auto f1vs = get_face_vertices(f1);
        auto f2vs = get_face_vertices(f2);
        auto f3vs = get_face_vertices(f3);
        auto f4vs = get_face_vertices(f4);
        if (m_vertex_attribute[f1vs[0].vid(*this)].m_is_on_surface &&
            m_vertex_attribute[f1vs[1].vid(*this)].m_is_on_surface &&
            m_vertex_attribute[f1vs[2].vid(*this)].m_is_on_surface) {
            if (m_face_attribute[f1.fid(*this)].m_is_surface_fs) surface_fs.push_back(f1);
        }

        if (m_vertex_attribute[f2vs[0].vid(*this)].m_is_on_surface &&
            m_vertex_attribute[f2vs[1].vid(*this)].m_is_on_surface &&
            m_vertex_attribute[f2vs[2].vid(*this)].m_is_on_surface) {
            if (m_face_attribute[f2.fid(*this)].m_is_surface_fs) surface_fs.push_back(f2);
        }
        if (m_vertex_attribute[f3vs[0].vid(*this)].m_is_on_surface &&
            m_vertex_attribute[f3vs[1].vid(*this)].m_is_on_surface &&
            m_vertex_attribute[f3vs[2].vid(*this)].m_is_on_surface) {
            if (m_face_attribute[f3.fid(*this)].m_is_surface_fs) surface_fs.push_back(f3);
        }
        if (m_vertex_attribute[f4vs[0].vid(*this)].m_is_on_surface &&
            m_vertex_attribute[f4vs[1].vid(*this)].m_is_on_surface &&
            m_vertex_attribute[f4vs[2].vid(*this)].m_is_on_surface) {
            if (m_face_attribute[f4.fid(*this)].m_is_surface_fs) surface_fs.push_back(f4);
        }
    }

    // construct the graph by V and E
    // eliminate those edges that contains v
    size_t vid = v.vid(*this); // current vid
    std::vector<size_t> one_ring_surface_vertices;
    std::vector<std::pair<size_t, size_t>> one_ring_surface_edges;

    for (auto f : surface_fs) {
        auto vs = get_face_vertices(f);
        if (vs[0].vid(*this) != vid && vs[1].vid(*this) != vid && vs[2].vid(*this) != vid) continue;
        for (auto v_tuple : vs) {
            if (v_tuple.vid(*this) != vid) one_ring_surface_vertices.push_back(v_tuple.vid(*this));
        }
        if (vs[0].vid(*this) != vid && vs[1].vid(*this) != vid)
            one_ring_surface_edges.push_back(std::make_pair(vs[0].vid(*this), vs[1].vid(*this)));
        if (vs[0].vid(*this) != vid && vs[2].vid(*this) != vid)
            one_ring_surface_edges.push_back(std::make_pair(vs[0].vid(*this), vs[2].vid(*this)));
        if (vs[1].vid(*this) != vid && vs[2].vid(*this) != vid)
            one_ring_surface_edges.push_back(std::make_pair(vs[1].vid(*this), vs[2].vid(*this)));
    }

    wmtk::vector_unique(one_ring_surface_vertices);
    std::map<size_t, int> v_idx_map;
    for (int i = 0; i < one_ring_surface_vertices.size(); i++) {
        v_idx_map[one_ring_surface_vertices[i]] = i;
    }

    // adjacency matrix
    int m = one_ring_surface_vertices.size();
    bool** adj_mat = new bool*[m];
    for (int i = 0; i < m; i++) {
        adj_mat[i] = new bool[m];
    }

    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
            adj_mat[i][j] = false;
        }
    }

    for (auto e : one_ring_surface_edges) {
        adj_mat[v_idx_map[e.first]][v_idx_map[e.second]] = true;
        adj_mat[v_idx_map[e.second]][v_idx_map[e.first]] = true;
    }

    // count links
    int cnt_links = 0;

    // union find
    std::vector<int> parent(m);
    for (int i = 0; i < m; i++) {
        parent[i] = i;
    }

    for (int i = 0; i < m; i++) {
        for (int j = i + 1; j < m; j++) {
            if (adj_mat[i][j]) {
                union_uf(i, j, parent);
            }
        }
    }

    for (int i = 0; i < m; i++) {
        if (parent[i] == i) {
            cnt_links++;
        }
    }

    // delete adjacency matrix
    for (int i = 0; i < m; i++) delete[] adj_mat[i];
    delete[] adj_mat;

    // test code
    // if (cnt_links > 1) {
    //     std::cout << "----------------------------" << std::endl;
    //     std::cout << "vid: " << vid << std::endl;
    //     std::cout << "one ring vs: ";
    //     for (int i = 0; i < one_ring_surface_vertices.size(); i++) {
    //         std::cout << one_ring_surface_vertices[i] << ": "
    //                   << m_vertex_attribute[one_ring_surface_vertices[i]].m_is_on_surface << " ";
    //     }
    //     std::cout << std::endl;
    //     std::cout << "one ring edges: ";
    //     for (int i = 0; i < one_ring_surface_edges.size(); i++) {
    //         std::cout << one_ring_surface_edges[i].first << "-" <<
    //         one_ring_surface_edges[i].second
    //                   << " ";
    //     }
    //     std::cout << std::endl;
    // }

    return cnt_links;
}

// int count_edge_links(const Tuple& e)
// {
//     auto one_ring_tets = get_incident_tets_for_edge(e);
//     std::vector<Tuple> surface_fs;
//     for (auto t : one_ring_tets) {
//         Tuple f1 = t;
//         Tuple f2 = t.switch_face(*this);
//         Tuple f3 = t.switch_edge(*this).switch_face(*this);
//         Tuple f4 = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);
//         if (m_face_attribute[f1.fid(*this)].m_is_surface_fs) surface_fs.push_back(f1);
//         if (m_face_attribute[f2.fid(*this)].m_is_surface_fs) surface_fs.push_back(f2);
//         if (m_face_attribute[f3.fid(*this)].m_is_surface_fs) surface_fs.push_back(f3);
//         if (m_face_attribute[f4.fid(*this)].m_is_surface_fs) surface_fs.push_back(f4);
//     }

//     size_t vid1 = e.vid(*this);
//     size_t vid2 = e.switch_vertex(*this).vid(*this);
//     std::vector<size_t> one_ring_surface_vertices;
//     std::vector<std::pair<size_t, size_t>> one_ring_surface_edges;

//     for (auto f : surface_fs) {
//         auto vs = get_face_vertices(f);
//         for (auto v_tuple : vs) {
//             if (v_tuple.vid(*this) != vid1 && v_tuple.vid(*this) != vid2)
//                 one_ring_surface_vertices.push_back(v_tuple.vid(*this));
//         }
//         if (vs[0].vid(*this) != vid1 && vs[1].vid(*this) != vid1 && vs[0].vid(*this) != vid2 &&
//             vs[1].vid(*this) != vid2)
//             one_ring_surface_edges.push_back(std::make_pair(vs[0].vid(*this), vs[1].vid(*this)));
//         if (vs[0].vid(*this) != vid1 && vs[2].vid(*this) != vid1 && vs[0].vid(*this) != vid2 &&
//             vs[2].vid(*this) != vid2)
//             one_ring_surface_edges.push_back(std::make_pair(vs[0].vid(*this), vs[2].vid(*this)));
//         if (vs[1].vid(*this) != vid1 && vs[2].vid(*this) != vid1 && vs[1].vid(*this) != vid2 &&
//             vs[2].vid(*this) != vid2)
//             one_ring_surface_edges.push_back(std::make_pair(vs[1].vid(*this), vs[2].vid(*this)));
//     }

//     wmtk::vector_unique(one_ring_surface_vertices);
//     std::map<size_t, int> v_idx_map;
//     for (int i = 0; i < one_ring_surface_vertices.size(); i++) {
//         v_idx_map[one_ring_surface_vertices[i]] = i;
//     }

//     int m = one_ring_surface_vertices.size();
//     bool** adj_mat = new bool*[m];
//     for (int i = 0; i < m; i++) {
//         adj_mat[i] = new bool[m];
//     }

//     for (int i = 0; i < m; i++) {
//         for (int j = 0; j < m; j++) {
//             adj_mat[i][j] = false;
//         }
//     }

//     for (auto e : one_ring_surface_edges) {
//         adj_mat[v_idx_map[e.first]][v_idx_map[e.second]] = true;
//         adj_mat[v_idx_map[e.second]][v_idx_map[e.first]] = true;
//     }

//     // count links
//     int cnt_links = 0;

//     // union find
//     std::vector<int> parent(m);
//     for (int i = 0; i < m; i++) {
//         parent[i] = i;
//     }

//     for (int i = 0; i < m; i++) {
//         for (int j = i + 1; j < m; j++) {
//             if (adj_mat[i][j]) {
//                 union_uf(i, j, parent);
//             }
//         }
//     }

//     for (int i = 0; i < m; i++) {
//         if (parent[i] == i) {
//             cnt_links++;
//         }
//     }

//     // delete adjacency matrix
//     for (int i = 0; i < m; i++) delete[] adj_mat[i];
//     delete[] adj_mat;


//     return cnt_links;

//     return 0;
// }

int TetWildMesh::count_edge_links(const Tuple& e)
{
    size_t vid1 = e.vid(*this);
    size_t vid2 = e.switch_vertex(*this).vid(*this);
    auto tets = get_incident_tets_for_edge(e);
    std::vector<size_t> incident_surface_faces;
    for (auto t : tets) {
        std::vector<Tuple> f(4);
        f[0] = t;
        f[1] = t.switch_face(*this);
        f[2] = t.switch_edge(*this).switch_face(*this);
        f[3] = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);

        for (int i = 0; i < 4; i++) {
            auto fvs = get_face_vertices(f[i]);
            if (!(m_vertex_attribute[fvs[0].vid(*this)].m_is_on_surface &&
                  m_vertex_attribute[fvs[1].vid(*this)].m_is_on_surface &&
                  m_vertex_attribute[fvs[2].vid(*this)].m_is_on_surface))
                continue;
            if (!m_face_attribute[f[i].fid(*this)].m_is_surface_fs) continue;
            auto vs = get_face_vertices(f[i]);
            std::array<size_t, 3> vids = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
            if (std::find(vids.begin(), vids.end(), vid1) != vids.end() &&
                std::find(vids.begin(), vids.end(), vid2) != vids.end()) {
                incident_surface_faces.push_back(f[i].fid(*this));
            }
        }
    }

    wmtk::vector_unique(incident_surface_faces);

    return incident_surface_faces.size();
}

bool TetWildMesh::check_vertex_param_type()
{
    // std::ofstream file("missing_param_v.obj");
    bool flag = true;
    return flag;
}

int TetWildMesh::flood_fill()
{
    int current_id = 0;
    auto tets = get_tets();
    std::map<size_t, bool> visited;

    for (const Tuple& t : tets) {
        size_t tid = t.tid(*this);
        if (visited.find(tid) != visited.end()) continue;

        // std::cout << "for loop current id: " << current_id << std::endl;

        visited[tid] = true;

        m_tet_attribute[tid].part_id = current_id;

        auto f1 = t;
        auto f2 = t.switch_face(*this);
        auto f3 = t.switch_edge(*this).switch_face(*this);
        auto f4 = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);

        std::queue<Tuple> bfs_queue;

        if (!m_face_attribute[f1.fid(*this)].m_is_surface_fs) {
            // std::cout << "in 1" << std::endl;
            auto oppo_t = f1.switch_tetrahedron(*this);
            if (oppo_t.has_value()) {
                if (visited.find((*oppo_t).tid(*this)) == visited.end()) bfs_queue.push(*oppo_t);
            }
        }
        if (!m_face_attribute[f2.fid(*this)].m_is_surface_fs) {
            // std::cout << "in 2" << std::endl;
            auto oppo_t = f2.switch_tetrahedron(*this);
            if (oppo_t.has_value()) {
                if (visited.find((*oppo_t).tid(*this)) == visited.end()) bfs_queue.push(*oppo_t);
            }
        }
        if (!m_face_attribute[f3.fid(*this)].m_is_surface_fs) {
            // std::cout << "in 3" << std::endl;
            auto oppo_t = f3.switch_tetrahedron(*this);
            if (oppo_t.has_value()) {
                if (visited.find((*oppo_t).tid(*this)) == visited.end()) bfs_queue.push(*oppo_t);
            }
        }
        if (!m_face_attribute[f4.fid(*this)].m_is_surface_fs) {
            // std::cout << "in 4" << std::endl;
            auto oppo_t = f4.switch_tetrahedron(*this);
            if (oppo_t.has_value()) {
                if (visited.find((*oppo_t).tid(*this)) == visited.end()) bfs_queue.push(*oppo_t);
            }
        }

        // std::cout << "while loop current id: ";

        while (!bfs_queue.empty()) {
            auto tmp = bfs_queue.front();
            bfs_queue.pop();
            size_t tmp_id = tmp.tid(*this);
            if (visited.find(tmp_id) != visited.end()) continue;

            visited[tmp_id] = true;
            // std::cout << tmp_id << " ";

            m_tet_attribute[tmp_id].part_id = current_id;

            auto f_tmp1 = tmp;
            auto f_tmp2 = tmp.switch_face(*this);
            auto f_tmp3 = tmp.switch_edge(*this).switch_face(*this);
            auto f_tmp4 = tmp.switch_vertex(*this).switch_edge(*this).switch_face(*this);

            if (!m_face_attribute[f_tmp1.fid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp1.switch_tetrahedron(*this);
                if (oppo_t.has_value()) {
                    if (visited.find((*oppo_t).tid(*this)) == visited.end())
                        bfs_queue.push(*oppo_t);
                }
            }
            if (!m_face_attribute[f_tmp2.fid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp2.switch_tetrahedron(*this);
                if (oppo_t.has_value()) {
                    if (visited.find((*oppo_t).tid(*this)) == visited.end())
                        bfs_queue.push(*oppo_t);
                }
            }
            if (!m_face_attribute[f_tmp3.fid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp3.switch_tetrahedron(*this);
                if (oppo_t.has_value()) {
                    if (visited.find((*oppo_t).tid(*this)) == visited.end())
                        bfs_queue.push(*oppo_t);
                }
            }
            if (!m_face_attribute[f_tmp4.fid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp4.switch_tetrahedron(*this);
                if (oppo_t.has_value()) {
                    if (visited.find((*oppo_t).tid(*this)) == visited.end())
                        bfs_queue.push(*oppo_t);
                }
            }
        }

        // std::cout << std::endl;

        current_id++;
    }
    return current_id;
}

void TetWildMesh::save_paraview(const std::string& path, const bool use_hdf5)
{
    // consolidate_mesh();

    // flood fill
    // int num_parts = flood_fill();
    // std::cout << "flood fill parts: " << num_parts << std::endl;
    const auto vs = get_vertices();
    const auto tets = get_tets();
    const auto faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });

    Eigen::MatrixXd V(vert_capacity(), 3);
    Eigen::MatrixXi T(tet_capacity(), 4);
    Eigen::MatrixXi F(faces.size(), 3);

    V.setZero();
    T.setZero();
    F.setZero();

    Eigen::MatrixXd parts(tet_capacity(), 1);
    parts.setZero();
    Eigen::MatrixXd wn_input(tet_capacity(), 1);
    wn_input.setZero();
    Eigen::MatrixXd wn_tracked(tet_capacity(), 1);
    wn_tracked.setZero();
    Eigen::MatrixXd t_energy(tet_capacity(), 1);
    t_energy.setZero();
    Eigen::VectorXd v_sizing_field(vert_capacity());
    v_sizing_field.setZero();
    Eigen::VectorXd v_is_rounded(vert_capacity());
    v_is_rounded.setZero();
    Eigen::VectorXd v_is_on_surface(vert_capacity());
    v_is_on_surface.setZero();
    Eigen::VectorXd v_is_on_open_boundary(vert_capacity());
    v_is_on_open_boundary.setZero();

    int index = 0;
    for (const Tuple& t : tets) {
        size_t tid = t.tid(*this);
        parts(index, 0) = m_tet_attribute[tid].part_id;
        wn_input(index, 0) = m_tet_attribute[tid].m_winding_number_input;
        wn_tracked(index, 0) = m_tet_attribute[tid].m_winding_number_tracked;
        t_energy(index, 0) = std::cbrt(m_tet_attribute[tid].m_quality);

        const auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            T(index, j) = vs[j].vid(*this);
        }
        ++index;
    }

    for (size_t i = 0; i < faces.size(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            F(i, j) = faces[i][j];
        }
    }

    for (auto v : vs) {
        const auto vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
        v_sizing_field[vid] = m_vertex_attribute[vid].m_sizing_scalar;
        v_is_rounded[vid] = m_vertex_attribute[vid].m_is_rounded ? 1 : 0;
        v_is_on_surface[vid] = m_vertex_attribute[vid].m_is_on_surface ? 1 : 0;
        v_is_on_open_boundary[vid] = m_vertex_attribute[vid].m_is_on_open_boundary ? 1 : 0;
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    if (use_hdf5) {
        throw std::runtime_error("Cannot write HDF5");
        // writer = std::make_shared<paraviewo::HDF5VTUWriter>();
    } else {
        writer = std::make_shared<paraviewo::VTUWriter>();
    }

    const auto out_path = path + (use_hdf5 ? ".hdf" : ".vtu");

    writer->add_cell_field("part", parts);
    writer->add_cell_field("winding_number_input", wn_input);
    writer->add_cell_field("winding_number_tracked", wn_tracked);
    writer->add_cell_field("t_energy", t_energy);
    writer->add_field("sizing_field", v_sizing_field);
    writer->add_field("is_rounded", v_is_rounded);
    writer->add_field("on_surface", v_is_on_surface);
    writer->add_field("on_open_boundary", v_is_on_open_boundary);


    logger().info("Write {}", out_path);
    writer->write_mesh(out_path, V, T);

    // surface
    {
        const auto surf_out_path = path + "_surf.vtu";
        std::shared_ptr<paraviewo::ParaviewWriter> surf_writer;
        surf_writer = std::make_shared<paraviewo::VTUWriter>();
        surf_writer->add_field("sizing_field", v_sizing_field);
        surf_writer->add_field("is_rounded", v_is_rounded);
        surf_writer->add_field("on_surface", v_is_on_surface);
        surf_writer->add_field("on_open_boundary", v_is_on_open_boundary);

        logger().info("Write {}", surf_out_path);
        surf_writer->write_mesh(surf_out_path, V, F);
    }
}

void TetWildMesh::init_sizing_field()
{
    const double min_refine_scalar = m_params.l_min / m_params.l;

    const double R = m_params.l * 1.8;

    for (auto v : get_vertices()) {
        // compute the distance between a vertex and its Ring surfaces
        // one ring vertices
        double min_uv_dist = 100000;
        double hit_surface_flag = false;
        for (auto u : get_one_ring_vertices_for_vertex(v)) {
            if (!m_vertex_attribute[u.vid(*this)].m_is_on_surface) continue;
            hit_surface_flag = true;
            double uv_dist =
                (m_vertex_attribute[v.vid(*this)].m_posf - m_vertex_attribute[u.vid(*this)].m_posf)
                    .norm();
            if (uv_dist < min_uv_dist) min_uv_dist = uv_dist;
        }

        if (!hit_surface_flag) continue;

        // edges
        double min_ev_dist = 100000;
        auto tets = get_one_ring_tets_for_vertex(v);

        for (auto t : tets) {
            std::array<Tuple, 4> fs;
            fs[0] = t;
            fs[1] = t.switch_face(*this);
            fs[2] = t.switch_edge(*this).switch_face(*this);
            fs[3] = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);


            for (int i = 0; i < 4; i++) {
                const Tuple& f1 = fs[i];
                auto f1vs = get_face_vertices(fs[i]);
                if (m_vertex_attribute[f1vs[0].vid(*this)].m_is_on_surface &&
                    m_vertex_attribute[f1vs[1].vid(*this)].m_is_on_surface &&
                    m_vertex_attribute[f1vs[2].vid(*this)].m_is_on_surface) {
                    if (m_face_attribute[f1.fid(*this)].m_is_surface_fs) {
                        auto vs = f1vs;
                        double ev_dist1 = (m_vertex_attribute[vs[0].vid(*this)].m_posf -
                                           m_vertex_attribute[v.vid(*this)].m_posf)
                                              .cross(
                                                  m_vertex_attribute[vs[1].vid(*this)].m_posf -
                                                  m_vertex_attribute[v.vid(*this)].m_posf)
                                              .norm() /
                                          (m_vertex_attribute[vs[0].vid(*this)].m_posf -
                                           m_vertex_attribute[vs[1].vid(*this)].m_posf)
                                              .norm();

                        double ev_dist2 = (m_vertex_attribute[vs[0].vid(*this)].m_posf -
                                           m_vertex_attribute[v.vid(*this)].m_posf)
                                              .cross(
                                                  m_vertex_attribute[vs[2].vid(*this)].m_posf -
                                                  m_vertex_attribute[v.vid(*this)].m_posf)
                                              .norm() /
                                          (m_vertex_attribute[vs[0].vid(*this)].m_posf -
                                           m_vertex_attribute[vs[2].vid(*this)].m_posf)
                                              .norm();
                        double ev_dist3 = (m_vertex_attribute[vs[1].vid(*this)].m_posf -
                                           m_vertex_attribute[v.vid(*this)].m_posf)
                                              .cross(
                                                  m_vertex_attribute[vs[2].vid(*this)].m_posf -
                                                  m_vertex_attribute[v.vid(*this)].m_posf)
                                              .norm() /
                                          (m_vertex_attribute[vs[1].vid(*this)].m_posf -
                                           m_vertex_attribute[vs[2].vid(*this)].m_posf)
                                              .norm();
                        if (min_ev_dist < ev_dist1) min_ev_dist = ev_dist1;
                        if (min_ev_dist < ev_dist2) min_ev_dist = ev_dist2;
                        if (min_ev_dist < ev_dist3) min_ev_dist = ev_dist3;
                    }
                }
            }
        }

        //if (min_dist < m_params.l / 2) continue;

        // adjust sizing field
        double min_dist = (min_uv_dist < min_ev_dist) ? min_uv_dist : min_ev_dist;
        double refine_scalar = min_dist / m_params.l;

        m_vertex_attribute[v.vid(*this)].m_sizing_scalar =
            std::min(refine_scalar, m_vertex_attribute[v.vid(*this)].m_sizing_scalar);

        std::vector<bool> visited(vert_capacity(), false);
        std::queue<size_t> v_queue;

        for (auto u : get_one_ring_vertices_for_vertex(v)) {
            v_queue.push(u.vid(*this));
        }

        while (!v_queue.empty()) {
            size_t vid = v_queue.front();
            v_queue.pop();
            if (visited[vid]) continue;
            visited[vid] = true;
            double dist =
                (m_vertex_attribute[vid].m_posf - m_vertex_attribute[v.vid(*this)].m_posf).norm();
            if (dist > R) continue;
            m_vertex_attribute[vid].m_sizing_scalar = std::min(
                dist / R * (1 - refine_scalar) + refine_scalar,
                m_vertex_attribute[v.vid(*this)].m_sizing_scalar);

            auto vids = get_one_ring_vids_for_vertex(vid);
            for (size_t n_vid : vids) {
                if (visited[n_vid]) continue;
                v_queue.push(n_vid);
            }
        }
    }
}

} // namespace wmtk::components::tetwild