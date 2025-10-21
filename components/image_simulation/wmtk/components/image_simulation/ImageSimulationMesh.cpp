
#include "ImageSimulationMesh.h"

#include "wmtk/utils/Rational.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/concurrent_vector.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/fmt/bundled/format.h>
#include <tracy/Tracy.hpp>
#include <igl/predicates/predicates.h>
#include <igl/winding_number.h>
#include <igl/write_triangle_mesh.h>
#include <igl/Timer.h>
#include <igl/orientable_patches.h>
#include <wmtk/utils/EnableWarnings.hpp>
#include <wmtk/utils/GeoUtils.h>
// clang-format on

#include <paraviewo/VTUWriter.hpp>


#include <geogram/points/kd_tree.h>
#include <limits>

namespace {
static int debug_print_counter = 0;
}

namespace wmtk::components::image_simulation {


VertexAttributes::VertexAttributes(const Vector3r& p)
{
    m_pos = p;
    m_posf = to_double(p);
}

void ImageSimulationMesh::mesh_improvement(int max_its)
{
    ////preprocessing
    // TODO: refactor to eliminate repeated partition.
    //
    ZoneScopedN("meshimprovementmain");

    compute_vertex_partition_morton();

    // write_vtu(fmt::format("debug_{}.vtu", m_debug_print_counter++));

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
        auto [max_energy, avg_energy] = local_operations({{1, 1, 1, 1}});

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

std::tuple<double, double> ImageSimulationMesh::local_operations(
    const std::array<int, 4>& ops,
    bool collapse_limit_length)
{
    igl::Timer timer;

    std::tuple<double, double> energy;

    auto sanity_checks = [this]() {
        logger().info("Perform sanity checks...");
        const auto faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });
        for (const auto& verts : faces) {
            const auto p0 = m_vertex_attribute[verts[0]].m_posf;
            const auto p1 = m_vertex_attribute[verts[1]].m_posf;
            const auto p2 = m_vertex_attribute[verts[2]].m_posf;
            if (m_envelope->is_outside({{p0, p1, p2}})) {
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
                    vert_capacity(),
                    tet_capacity());
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
            wmtk::logger().info("split max energy = {} avg = {}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 1) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==collapsing {}==", n);
                collapse_all_edges(collapse_limit_length);
                wmtk::logger().info(
                    "#vertices {}, #tets {} after collapse",
                    vert_capacity(),
                    tet_capacity());
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
            wmtk::logger().info("collapse max energy = {} avg = {}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 2) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==swapping {}==", n);
                swap_all_edges_44();
                swap_all_edges();
                swap_all_faces();
            }
            // save_paraview(fmt::format("debug_{}", debug_print_counter++), false);
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("swap max energy = {} avg = {}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 3) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==smoothing {}==", n);
                smooth_all_vertices();
            }
            // save_paraview(fmt::format("debug_{}", debug_print_counter++), false);
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("smooth max energy = {} avg = {}", max_energy, avg_energy);
            sanity_checks();
        }
        // output_faces(fmt::format("out-op{}.obj", i), [](auto& f) { return f.m_is_surface_fs; });
    }
    // save_paraview(fmt::format("debug_{}", debug_print_counter++), false);
    energy = get_max_avg_energy();
    wmtk::logger().info("max energy = {}", std::get<0>(energy));
    wmtk::logger().info("avg energy = {}", std::get<1>(energy));
    wmtk::logger().info("time = {}", timer.getElapsedTime());


    return energy;
}

bool ImageSimulationMesh::adjust_sizing_field(double max_energy)
{
    wmtk::logger().info("#vertices {}, #tets {}", vert_capacity(), tet_capacity());

    const double stop_filter_energy = m_params.stop_energy * 0.8;
    double filter_energy = std::max(max_energy / 100, stop_filter_energy);
    filter_energy = std::min(filter_energy, 100.);

    const auto recover_scalar = 1.5;
    const auto refine_scalar = 0.5;
    const auto min_refine_scalar = m_params.l_min / m_params.l;

    // outputs scale_multipliers
    tbb::concurrent_vector<double> scale_multipliers(vert_capacity(), recover_scalar);

    tbb::concurrent_vector<Vector3d> pts;
    tbb::concurrent_queue<size_t> v_queue;
    TetMesh::for_each_tetra([&](auto& t) {
        auto tid = t.tid(*this);
        if (std::cbrt(m_tet_attribute[tid].m_quality) < filter_energy) return;
        auto vs = oriented_tet_vids(t);
        Vector3d c(0, 0, 0);
        for (int j = 0; j < 4; j++) {
            c += (m_vertex_attribute[vs[j]].m_posf);
            v_queue.emplace(vs[j]);
            std::cout << vs[j] << " ";
        }
        pts.emplace_back(c / 4);
    });

    std::cout << std::endl;

    wmtk::logger().info("filter energy {} Low Quality Tets {}", filter_energy, pts.size());

    // debug code
    std::queue<size_t> v_queue_serial;
    for (tbb::concurrent_queue<size_t>::const_iterator i(v_queue.unsafe_begin());
         i != v_queue.unsafe_end();
         ++i) {
        // std::cout << *i << " ";
        v_queue_serial.push(*i);
    }
    std::cout << std::endl;

    const double R = m_params.l * 1.8;

    int sum = 0;
    int adjcnt = 0;

    std::vector<bool> visited(vert_capacity(), false);

    GEO::NearestNeighborSearch_var nnsearch = GEO::NearestNeighborSearch::create(3, "BNN");
    nnsearch->set_points(pts.size(), pts[0].data());

    std::vector<size_t> cache_one_ring;
    // size_t vid;

    while (!v_queue_serial.empty()) {
        // std::cout << vid << " ";
        sum++;
        size_t vid = v_queue_serial.front();
        std::cout << vid << " ";
        v_queue_serial.pop();
        if (visited[vid]) continue;
        visited[vid] = true;
        adjcnt++;

        auto& pos_v = m_vertex_attribute[vid].m_posf;
        auto sq_dist = 0.;
        GEO::index_t _1;
        nnsearch->get_nearest_neighbors(1, pos_v.data(), &_1, &sq_dist);
        auto dist = std::sqrt(std::max(sq_dist, 0.)); // compute dist(pts, pos_v);
        // std::cout << dist << " ";

        if (dist > R) { // outside R-ball, unmark.
            continue;
        }

        scale_multipliers[vid] = std::min(
            scale_multipliers[vid],
            dist / R * (1 - refine_scalar) + refine_scalar); // linear interpolate

        auto vids = get_one_ring_vids_for_vertex_adj(vid, cache_one_ring);
        for (size_t n_vid : vids) {
            if (visited[n_vid]) continue;
            v_queue_serial.push(n_vid);
        }
    }

    std::cout << std::endl;

    std::cout << sum << " " << adjcnt << std::endl;
    // while (v_queue.try_pop(vid)) {
    //     // std::cout << vid << " ";
    //     sum++;
    //     // size_t vid = v_queue.front();
    //     // v_queue.pop();
    //     if (visited[vid]) continue;
    //     visited[vid] = true;
    //     adjcnt++;

    //     auto& pos_v = m_vertex_attribute[vid].m_posf;
    //     auto sq_dist = 0.;
    //     GEO::index_t _1;
    //     nnsearch->get_nearest_neighbors(1, pos_v.data(), &_1, &sq_dist);
    //     auto dist = std::sqrt(std::max(sq_dist, 0.)); // compute dist(pts, pos_v);

    //     if (dist > R) { // outside R-ball, unmark.
    //         continue;
    //     }

    //     scale_multipliers[vid] = std::min(
    //         scale_multipliers[vid],
    //         dist / R * (1 - refine_scalar) + refine_scalar); // linear interpolate

    //     auto vids = get_one_ring_vids_for_vertex_adj(vid, cache_one_ring);
    //     for (size_t n_vid : vids) {
    //         if (visited[n_vid]) continue;
    //         v_queue.emplace(n_vid);
    //     }
    // }

    std::atomic_bool is_hit_min_edge_length = false;
    for_each_vertex([&](auto& v) {
        auto vid = v.vid(*this);
        std::cout << vid << " ";
        auto& v_attr = m_vertex_attribute[vid];

        auto new_scale = v_attr.m_sizing_scalar * scale_multipliers[vid];
        if (new_scale > 1)
            v_attr.m_sizing_scalar = 1;
        else if (new_scale < min_refine_scalar) {
            is_hit_min_edge_length = true;
            v_attr.m_sizing_scalar = min_refine_scalar;
        } else
            v_attr.m_sizing_scalar = new_scale;
    });
    std::cout << std::endl;


    return is_hit_min_edge_length.load();
}

bool ImageSimulationMesh::adjust_sizing_field_serial(double max_energy)
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

    GEO::NearestNeighborSearch_var nnsearch = GEO::NearestNeighborSearch::create(3, "BNN");
    nnsearch->set_points(pts.size(), pts[0].data());

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
        auto sq_dist = 0.;
        GEO::index_t _1;
        nnsearch->get_nearest_neighbors(1, pos_v.data(), &_1, &sq_dist);
        auto dist = std::sqrt(std::max(sq_dist, 0.)); // compute dist(pts, pos_v);
        // std::cout << dist << " ";

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

/////////////////////////////////////////////////////////////////////
void ImageSimulationMesh::output_faces(
    std::string file,
    std::function<bool(const FaceAttributes&)> cond)
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


double ImageSimulationMesh::get_length2(const Tuple& l) const
{
    SmartTuple v1(*this, l);
    SmartTuple v2 = v1.switch_vertex();
    double length =
        (m_vertex_attribute[v1.vid()].m_posf - m_vertex_attribute[v2.vid()].m_posf).squaredNorm();
    return length;
}

void ImageSimulationMesh::write_msh(std::string file)
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

    msh.add_tet_attribute<1>("tag", [&](size_t i) { return m_tet_attribute[i].tag; });

    msh.save(file, true);
}

std::tuple<double, double> ImageSimulationMesh::get_max_avg_energy()
{
    double max_energy = -1.;
    double avg_energy = 0.;
    auto cnt = 0;

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

bool ImageSimulationMesh::is_inverted_f(const Tuple& loc) const
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

bool ImageSimulationMesh::is_inverted(const Tuple& loc) const
{
    // Return a positive value if the point pd lies below the
    // plane passing through pa, pb, and pc; "below" is defined so
    // that pa, pb, and pc appear in counterclockwise order when
    // viewed from above the plane.

    auto vs = oriented_tet_vertices(loc);

    //
    if (m_vertex_attribute[vs[0].vid(*this)].m_is_rounded &&
        m_vertex_attribute[vs[1].vid(*this)].m_is_rounded &&
        m_vertex_attribute[vs[2].vid(*this)].m_is_rounded &&
        m_vertex_attribute[vs[3].vid(*this)].m_is_rounded) {
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
    } else {
        Vector3r n = ((m_vertex_attribute[vs[1].vid(*this)].m_pos) -
                      m_vertex_attribute[vs[0].vid(*this)].m_pos)
                         .cross(
                             (m_vertex_attribute[vs[2].vid(*this)].m_pos) -
                             m_vertex_attribute[vs[0].vid(*this)].m_pos);
        Vector3r d = (m_vertex_attribute[vs[3].vid(*this)].m_pos) -
                     m_vertex_attribute[vs[0].vid(*this)].m_pos;
        auto res = n.dot(d);
        if (res > 0) // predicates returns pos value: non-inverted
            return false;
        else
            return true;
    }
}

bool ImageSimulationMesh::round(const Tuple& v)
{
    const size_t vid = v.vid(*this);
    auto& va = m_vertex_attribute[vid];
    if (va.m_is_rounded) {
        return true;
    }

    const Vector3r old_pos = va.m_pos;
    va.m_pos = to_rational(va.m_posf);

    const auto tets = get_one_ring_tets_for_vertex(v);
    for (const Tuple& tet : tets) {
        if (is_inverted(tet)) {
            va.m_pos = old_pos;
            return false;
        }
    }

    va.m_is_rounded = true;
    return true;
}

double ImageSimulationMesh::get_quality(const Tuple& loc) const
{
    std::array<Vector3d, 4> ps;
    auto its = oriented_tet_vids(loc);
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


bool ImageSimulationMesh::invariants(const std::vector<Tuple>& tets)
{
    return true;
}

std::vector<std::array<size_t, 3>> ImageSimulationMesh::get_faces_by_condition(
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

bool ImageSimulationMesh::is_edge_on_surface(const Tuple& loc)
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


bool ImageSimulationMesh::is_edge_on_bbox(const Tuple& loc)
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

bool ImageSimulationMesh::check_attributes()
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
            bool is_out = m_envelope->is_outside(
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
            bool is_out = m_envelope->is_outside(m_vertex_attribute[i].m_posf);
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

int ImageSimulationMesh::count_vertex_links(const Tuple& v)
{
    // get one ring faces on surface
    const auto one_ring_tets = get_one_ring_tets_for_vertex(v);
    std::vector<Tuple> surface_fs;

    const auto v_on_surf = [this](const Tuple& t) {
        return m_vertex_attribute[t.vid(*this)].m_is_on_surface;
    };

    for (const Tuple& t : one_ring_tets) {
        Tuple f1 = t;
        Tuple f2 = t.switch_face(*this);
        Tuple f3 = t.switch_edge(*this).switch_face(*this);
        Tuple f4 = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);

        const auto f1vs = get_face_vertices(f1);
        if (v_on_surf(f1vs[0]) && v_on_surf(f1vs[1]) && v_on_surf(f1vs[2])) {
            if (m_face_attribute[f1.fid(*this)].m_is_surface_fs) {
                surface_fs.push_back(f1);
            }
        }
        const auto f2vs = get_face_vertices(f2);
        if (v_on_surf(f2vs[0]) && v_on_surf(f2vs[1]) && v_on_surf(f2vs[2])) {
            if (m_face_attribute[f2.fid(*this)].m_is_surface_fs) {
                surface_fs.push_back(f2);
            }
        }
        const auto f3vs = get_face_vertices(f3);
        if (v_on_surf(f3vs[0]) && v_on_surf(f3vs[1]) && v_on_surf(f3vs[2])) {
            if (m_face_attribute[f3.fid(*this)].m_is_surface_fs) {
                surface_fs.push_back(f3);
            }
        }
        const auto f4vs = get_face_vertices(f4);
        if (v_on_surf(f4vs[0]) && v_on_surf(f4vs[1]) && v_on_surf(f4vs[2])) {
            if (m_face_attribute[f4.fid(*this)].m_is_surface_fs) {
                surface_fs.push_back(f4);
            }
        }
    }

    // surface_fs now holds all faces in the one ring that are on the surface

    // construct the graph by V and E
    // eliminate those edges that contains v
    const size_t vid = v.vid(*this); // current vid
    std::vector<size_t> one_ring_surface_vertices;
    std::vector<std::pair<size_t, size_t>> one_ring_surface_edges;

    for (const Tuple& f : surface_fs) {
        const auto vs = get_face_vertices(f);
        const std::array<size_t, 3> vids = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
        if (vids[0] != vid && vids[1] != vid && vids[2] != vid) {
            // ignore faces that are not incident to vid
            continue;
        }
        for (const size_t vv : vids) {
            if (vv != vid) {
                one_ring_surface_vertices.emplace_back(vv);
            }
        }
        if (vids[0] != vid && vids[1] != vid) {
            one_ring_surface_edges.emplace_back(vids[0], vids[1]);
        }
        if (vids[0] != vid && vids[2] != vid) {
            one_ring_surface_edges.emplace_back(vids[0], vids[2]);
        }
        if (vids[1] != vid && vids[2] != vid) {
            one_ring_surface_edges.emplace_back(vids[1], vids[2]);
        }
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

    for (const auto& e : one_ring_surface_edges) {
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

    return cnt_links;
}

int ImageSimulationMesh::count_edge_links(const Tuple& e)
{
    const size_t vid1 = e.vid(*this);
    const size_t vid2 = e.switch_vertex(*this).vid(*this);
    const auto tets = get_incident_tets_for_edge(e);
    std::vector<size_t> incident_surface_faces;
    for (const Tuple& t : tets) {
        std::array<Tuple, 4> f;
        f[0] = t;
        f[1] = t.switch_face(*this);
        f[2] = t.switch_edge(*this).switch_face(*this);
        f[3] = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);

        for (int i = 0; i < 4; i++) {
            const auto vs = get_face_vertices(f[i]);
            std::array<size_t, 3> vids = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
            if (!(m_vertex_attribute[vids[0]].m_is_on_surface &&
                  m_vertex_attribute[vids[1]].m_is_on_surface &&
                  m_vertex_attribute[vids[2]].m_is_on_surface)) {
                continue;
            }
            const size_t fid = f[i].fid(*this);
            if (!m_face_attribute[fid].m_is_surface_fs) {
                continue;
            }
            if (std::find(vids.begin(), vids.end(), vid1) != vids.end() &&
                std::find(vids.begin(), vids.end(), vid2) != vids.end()) {
                incident_surface_faces.push_back(fid);
            }
        }
    }

    wmtk::vector_unique(incident_surface_faces);

    return incident_surface_faces.size();
}

int ImageSimulationMesh::flood_fill()
{
    int current_id = 0;
    auto tets = get_tets();
    std::map<size_t, bool> visited;

    for (const Tuple& t : tets) {
        size_t tid = t.tid(*this);
        if (visited.find(tid) != visited.end()) continue;

        visited[tid] = true;

        m_tet_attribute[tid].part_id = current_id;

        auto f1 = t;
        auto f2 = t.switch_face(*this);
        auto f3 = t.switch_edge(*this).switch_face(*this);
        auto f4 = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);

        std::queue<Tuple> bfs_queue;

        if (!m_face_attribute[f1.fid(*this)].m_is_surface_fs) {
            auto oppo_t = f1.switch_tetrahedron(*this);
            if (oppo_t.has_value()) {
                if (visited.find((*oppo_t).tid(*this)) == visited.end()) bfs_queue.push(*oppo_t);
            }
        }
        if (!m_face_attribute[f2.fid(*this)].m_is_surface_fs) {
            auto oppo_t = f2.switch_tetrahedron(*this);
            if (oppo_t.has_value()) {
                if (visited.find((*oppo_t).tid(*this)) == visited.end()) bfs_queue.push(*oppo_t);
            }
        }
        if (!m_face_attribute[f3.fid(*this)].m_is_surface_fs) {
            auto oppo_t = f3.switch_tetrahedron(*this);
            if (oppo_t.has_value()) {
                if (visited.find((*oppo_t).tid(*this)) == visited.end()) bfs_queue.push(*oppo_t);
            }
        }
        if (!m_face_attribute[f4.fid(*this)].m_is_surface_fs) {
            auto oppo_t = f4.switch_tetrahedron(*this);
            if (oppo_t.has_value()) {
                if (visited.find((*oppo_t).tid(*this)) == visited.end()) bfs_queue.push(*oppo_t);
            }
        }

        while (!bfs_queue.empty()) {
            auto tmp = bfs_queue.front();
            bfs_queue.pop();
            size_t tmp_id = tmp.tid(*this);
            if (visited.find(tmp_id) != visited.end()) continue;

            visited[tmp_id] = true;

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

        current_id++;
    }
    return current_id;
}

void ImageSimulationMesh::write_vtu(const std::string& path)
{
    consolidate_mesh();
    logger().info("Write {}", path);
    const auto& vs = get_vertices();
    const auto& tets = get_tets();

    Eigen::MatrixXd V(vs.size(), 3);
    Eigen::MatrixXi T(tets.size(), 4);

    Eigen::MatrixXd parts(tets.size(), 1);
    Eigen::MatrixXd tags(tets.size(), 1);
    Eigen::MatrixXd amips(tets.size(), 1);

    int index = 0;
    for (const Tuple& t : tets) {
        size_t tid = t.tid(*this);
        parts(index, 0) = m_tet_attribute[tid].part_id;
        tags(index, 0) = m_tet_attribute[tid].tag;
        amips(index, 0) = std::cbrt(m_tet_attribute[tid].m_quality);

        const auto& vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            T(index, j) = vs[j].vid(*this);
        }
        ++index;
    }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();

    writer->add_cell_field("part", parts);
    writer->add_cell_field("tag", tags);
    writer->add_cell_field("quality", amips);
    writer->write_mesh(path, V, T);
}

void ImageSimulationMesh::write_surface(const std::string& path) const
{
    std::vector<std::array<size_t, 3>> outface;
    for (const Tuple& f : get_faces()) {
        if (!m_face_attribute[f.fid(*this)].m_is_surface_fs) {
            continue;
        }
        const auto verts = get_face_vertices(f);
        std::array<size_t, 3> vids = {
            {verts[0].vid(*this), verts[1].vid(*this), verts[2].vid(*this)}};
        outface.emplace_back(vids);
    }
    Eigen::MatrixXd matV = Eigen::MatrixXd::Zero(vert_capacity(), 3);
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        matV.row(vid) = m_vertex_attribute[vid].m_posf;
    }
    Eigen::MatrixXi matF(outface.size(), 3);
    for (size_t i = 0; i < outface.size(); i++) {
        matF.row(i) << outface[i][0], outface[i][1], outface[i][2];
    }
    igl::write_triangle_mesh(path, matV, matF);

    wmtk::logger().info("Output face size {}", outface.size());
}

void ImageSimulationMesh::init_sizing_field()
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

} // namespace wmtk::components::image_simulation