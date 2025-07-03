
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

//#include <paraviewo/HDF5VTUWriter.hpp>
#include <paraviewo/VTUWriter.hpp>


#include <geogram/points/kd_tree.h>
#include <limits>

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
        } else
            m = 0;
        if (is_hit_min_edge_length) {
            // todo: maybe to do sth
        }
        pre_max_energy = max_energy;
        pre_avg_energy = avg_energy;
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
        } else if (i == 1) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==collapsing {}==", n);
                collapse_all_edges();
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
        } else if (i == 2) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==swapping {}==", n);
                swap_all_edges_44();
                swap_all_edges();
                swap_all_faces();
            }
        } else if (i == 3) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==smoothing {}==", n);
                smooth_all_vertices();
            }
        }
        // output_faces(fmt::format("out-op{}.obj", i), [](auto& f) { return f.m_is_surface_fs; });
    }
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
    size_t vid;

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
    size_t vid;

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

void ImageSimulationMesh::filter_outside(
    const std::vector<Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces,
    bool remove_ouside)
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
        wmtk::logger().info("Output face size {}", outface.size());
        auto F0 = F;
        Eigen::VectorXi C;
        bfs_orient(F0, F, C);
        wmtk::logger().info("BFS orient {}", F.rows());
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
        wmtk::logger().info("Correcting");
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
    {
        const auto tets = get_tets();
        for (int i = 0; i < tets.size(); ++i) {
            const size_t tid = tets[i].tid(*this);
            m_tet_attribute[tid].m_winding_number = W(i);
        }
    }

    wmtk::logger().info("Removing...");

    std::vector<size_t> rm_tids;
    for (int i = 0; i < W.rows(); i++) {
        if (W(i) <= 0.5) {
            if (remove_ouside) {
                rm_tids.push_back(tets[i].tid(*this));
            }
        }
    }

    if (remove_ouside) remove_tets_by_ids(rm_tids);
}

/////////////////////////////////////////////////////////////////////
void ImageSimulationMesh::output_faces(
    std::string file,
    std::function<bool(const FaceAttributes&)> cond)
{
    auto outface = get_faces_by_condition(cond);
    Eigen::MatrixXd matV = Eigen::MatrixXd::Zero(vert_capacity(), 3);
    for (auto v : get_vertices()) {
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


void ImageSimulationMesh::output_mesh(std::string file)
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
    msh.add_tet_attribute<1>("winding_number", [&](size_t i) {
        return std::cbrt(m_tet_attribute[i].m_winding_number);
    });

    msh.save(file, true);
}


double ImageSimulationMesh::get_length2(const wmtk::TetMesh::Tuple& l) const
{
    auto& m = *this;
    auto& v1 = l;
    auto v2 = l.switch_vertex(m);
    double length =
        (m.m_vertex_attribute[v1.vid(m)].m_posf - m.m_vertex_attribute[v2.vid(m)].m_posf)
            .squaredNorm();
    return length;
}

std::tuple<double, double> ImageSimulationMesh::get_max_avg_energy()
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
        auto vs = oriented_tet_vertices(tup);

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
            res.emplace_back(
                std::array<size_t, 3>{
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

long long ImageSimulationMesh::checksum_vidx()
{
    long long checksum = 0;
    auto vs = get_vertices();
    for (int i = 0; i < vs.size(); i++) {
        if (vs[i].is_valid(*this)) checksum += i;
    }
    return checksum;
}

long long ImageSimulationMesh::checksum_tidx()
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

int ImageSimulationMesh::count_vertex_links(const Tuple& v)
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

int ImageSimulationMesh::count_edge_links(const Tuple& e)
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


bool ImageSimulationMesh::is_triangle_coplanar_collection(
    const Vector3r& v1,
    const Vector3r& v2,
    const Vector3r& v3,
    const coplanar_triangle_collection& collection)
{
    // // check normal (exact)
    // Vector3r triangle_normal = (v2 - v1).cross(v3 - v1);
    // if (triangle_normal.cross(collection.normal) != Vector3r(0, 0, 0)) return false;

    // // check coplanar
    // Vector3r vec = v1 - collection.a_pos;
    // if (vec.dot(collection.normal) != 0) return false; // dot?

    int o1 = orient3d_t(collection.a_pos, collection.b_pos, collection.c_pos, v1);
    int o2 = orient3d_t(collection.a_pos, collection.b_pos, collection.c_pos, v2);
    int o3 = orient3d_t(collection.a_pos, collection.b_pos, collection.c_pos, v3);

    return o1 == 0 && o2 == 0 && o3 == 0;
    // return true;
}

bool ImageSimulationMesh::is_triangle_nearly_coplanar_collection(
    const Vector3r& v1,
    const Vector3r& v2,
    const Vector3r& v3,
    const coplanar_triangle_collection& collection,
    double theta = 2)
{
    // // check normal (exact)
    // theta is in degree
    Vector3d tri_normal = to_double((v1 - v2).cross(v1 - v3));
    Vector3d collection_normal = to_double(collection.normal);
    if (tri_normal.cross(collection_normal).norm() /
            (tri_normal.norm() * collection_normal.norm()) <
        std::sin(theta / 180 * M_PI))
        return true;

    return false;
}

std::vector<std::vector<size_t>> ImageSimulationMesh::transfer_vf_to_face_face_connectivity(
    size_t num_v,
    std::vector<std::array<size_t, 3>> faces)
{
    struct v_conn
    {
        std::vector<size_t> conn_faces;
    };

    std::vector<v_conn> vertex_connectivity(num_v);

    for (int i = 0; i < faces.size(); i++) {
        for (int j = 0; j < 3; j++) {
            vertex_connectivity[faces[i][j]].conn_faces.push_back(i);
        }
    }

    std::vector<std::vector<size_t>> face_connectivity(faces.size());
    for (int i = 0; i < faces.size(); i++) {
        size_t v1 = faces[i][0];
        size_t v2 = faces[i][1];
        size_t v3 = faces[i][2];

        auto edge_conn_1 = wmtk::set_intersection(
            vertex_connectivity[v1].conn_faces,
            vertex_connectivity[v2].conn_faces);

        auto edge_conn_2 = wmtk::set_intersection(
            vertex_connectivity[v1].conn_faces,
            vertex_connectivity[v3].conn_faces);

        auto edge_conn_3 = wmtk::set_intersection(
            vertex_connectivity[v2].conn_faces,
            vertex_connectivity[v3].conn_faces);

        for (int k = 0; k < edge_conn_1.size(); k++) {
            if (edge_conn_1[k] != i) face_connectivity[i].push_back(edge_conn_1[k]);
        }

        for (int k = 0; k < edge_conn_2.size(); k++) {
            if (edge_conn_2[k] != i) face_connectivity[i].push_back(edge_conn_2[k]);
        }

        for (int k = 0; k < edge_conn_3.size(); k++) {
            if (edge_conn_3[k] != i) face_connectivity[i].push_back(edge_conn_3[k]);
        }
    }

    return face_connectivity;
}

void ImageSimulationMesh::detect_coplanar_triangle_collections(
    const std::vector<Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces)
{
    // transfer to rational coords
    std::vector<Vector3r> vertices_rational(vertices.size());
    for (size_t i = 0; i < vertices.size(); i++) {
        vertices_rational[i] = to_rational(vertices[i]);
    }

    // get face-face connectivity graph
    std::vector<std::vector<size_t>> face_adj_list =
        transfer_vf_to_face_face_connectivity(vertices.size(), faces);

    // debug code
    // for (int i = 0; i < face_adj_list.size(); i++) {
    //     std::cout << i << ": ";
    //     for (int j = 0; j < face_adj_list[i].size(); j++) {
    //         std::cout << face_adj_list[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // bfs to get the collection
    std::vector<ImageSimulationMesh::coplanar_triangle_collection> collections;
    std::vector<bool> visited_face(faces.size(), false); // visited if is already in a collection

    for (size_t i = 0; i < faces.size(); i++) {
        if (visited_face[i]) continue;
        visited_face[i] = true;

        // init a new collection with face[i]
        coplanar_triangle_collection collection;
        collection.face_ids.push_back(i);
        collection.a_pos = vertices_rational[faces[i][0]];
        collection.b_pos = vertices_rational[faces[i][1]];
        collection.c_pos = vertices_rational[faces[i][2]];
        collection.normal =
            (collection.a_pos - collection.b_pos).cross(collection.a_pos - collection.c_pos);
        collection.param_u = collection.b_pos - collection.a_pos;
        collection.param_v = collection.normal.cross(collection.param_v);

        // bfs
        std::queue<size_t> bfs_queue;
        for (int j = 0; j < face_adj_list[i].size(); j++) {
            if (!visited_face[face_adj_list[i][j]]) bfs_queue.push(face_adj_list[i][j]);
        }
        while (!bfs_queue.empty()) {
            size_t fid = bfs_queue.front();
            bfs_queue.pop();
            // std::cout << "faces.size(): " << faces.size() << std::endl;
            // std::cout << "fid: " << fid << std::endl;
            if (visited_face[fid]) continue;

            // check if nearly coplanar
            if (is_triangle_coplanar_collection(
                    vertices_rational[faces[fid][0]],
                    vertices_rational[faces[fid][1]],
                    vertices_rational[faces[fid][2]],
                    collection)) {
                visited_face[fid] = true;
                collection.face_ids.push_back(fid);
                for (int j = 0; j < face_adj_list[fid].size(); j++) {
                    if (!visited_face[face_adj_list[fid][j]]) bfs_queue.push(face_adj_list[fid][j]);
                }
            }
        }
        if (collection.face_ids.size() == 1) {
            collection.effective = false;
        } else {
            collection.effective = true;
        }
        collections.push_back(collection);
    }
    // return collections;
    // pass to image_simulation;

    // test code
    for (int i = 0; i < visited_face.size(); i++) {
        if (!visited_face[i])
            std::cout << "INPUT TRIANGLE NOT INSERT INTO ANY COLLECTION!!!" << std::endl;
    }

    triangle_collections_from_input_surface.input_vertices_rational = vertices_rational;
    triangle_collections_from_input_surface.input_faces = faces;
    triangle_collections_from_input_surface.collections = collections;

    // detect nearly coplanar collections
    std::vector<ImageSimulationMesh::coplanar_triangle_collection> collections_nearly;
    std::vector<bool> visited_face_nearly(
        faces.size(),
        false); // visited if is already in a collection

    for (size_t i = 0; i < faces.size(); i++) {
        if (visited_face_nearly[i]) continue;
        visited_face_nearly[i] = true;

        // init a new collection with face[i]
        coplanar_triangle_collection collection;

        collection.face_ids.push_back(i);
        collection.a_pos = vertices_rational[faces[i][0]];
        collection.b_pos = vertices_rational[faces[i][1]];
        collection.c_pos = vertices_rational[faces[i][2]];
        collection.normal =
            (collection.a_pos - collection.b_pos).cross(collection.a_pos - collection.c_pos);
        collection.param_u = collection.b_pos - collection.a_pos;
        collection.param_v = collection.normal.cross(collection.param_v);

        collection.normal_f = to_double(collection.normal).normalized();
        collection.a_pos_f = to_double(collection.a_pos);
        collection.param_u_f = to_double(collection.param_u).normalized();
        collection.param_v_f = to_double(collection.param_v).normalized();


        // bfs
        std::queue<size_t> bfs_queue;
        for (int j = 0; j < face_adj_list[i].size(); j++) {
            if (!visited_face_nearly[face_adj_list[i][j]]) bfs_queue.push(face_adj_list[i][j]);
        }
        while (!bfs_queue.empty()) {
            size_t fid = bfs_queue.front();
            bfs_queue.pop();
            // std::cout << "faces.size(): " << faces.size() << std::endl;
            // std::cout << "fid: " << fid << std::endl;
            if (visited_face_nearly[fid]) continue;

            // check if nearly coplanar
            if (is_triangle_nearly_coplanar_collection(
                    vertices_rational[faces[fid][0]],
                    vertices_rational[faces[fid][1]],
                    vertices_rational[faces[fid][2]],
                    collection,
                    2)) {
                visited_face_nearly[fid] = true;
                collection.face_ids.push_back(fid);
                for (int j = 0; j < face_adj_list[fid].size(); j++) {
                    if (!visited_face_nearly[face_adj_list[fid][j]])
                        bfs_queue.push(face_adj_list[fid][j]);
                }
            }
        }
        if (collection.face_ids.size() == 1) {
            collection.effective = false;
        } else {
            collection.effective = true;
        }
        collections_nearly.push_back(collection);
    }
    // return collections;
    // pass to image_simulation;

    // test code
    for (int i = 0; i < visited_face_nearly.size(); i++) {
        if (!visited_face_nearly[i])
            std::cout << "INPUT TRIANGLE NOT INSERT INTO ANY NEAR COPLANAR COLLECTION!!!"
                      << std::endl;
    }

    triangle_collections_from_input_surface.nearly_coplanar_collections = collections_nearly;

    std::vector<size_t> exact_to_nearly_map(collections.size());
    // get correspondence exact to nealy;
    for (int i = 0; i < collections.size(); i++) {
        size_t member_fid = collections[i].face_ids[0];
        for (int j = 0; j < collections_nearly.size(); j++) {
            if (std::find(
                    collections_nearly[j].face_ids.begin(),
                    collections_nearly[j].face_ids.end(),
                    member_fid) != collections_nearly[j].face_ids.end()) {
                exact_to_nearly_map[i] = j;
                break;
            }
        }
    }

    triangle_collections_from_input_surface.exact_to_nearly_map = exact_to_nearly_map;

    // debug code
    for (int i = 0; i < collections.size(); i++) {
        std::cout << "collection " << i << ": ";
        for (int j = 0; j < collections[i].face_ids.size(); j++) {
            std::cout << collections[i].face_ids[j] << " ";
        }
        std::cout << std::endl;
    }

    for (int i = 0; i < collections_nearly.size(); i++) {
        std::cout << "nearly collection " << i << ": ";
        for (int j = 0; j < collections_nearly[i].face_ids.size(); j++) {
            std::cout << collections_nearly[i].face_ids[j] << " ";
        }
        std::cout << std::endl;
    }

    // for (int i = 0; i < exact_to_nearly_map.size(); i++) {
    //     std::cout << "exact to nearly map " << i << " --> " << exact_to_nearly_map[i] <<
    //     std::endl;
    // }
}

bool ImageSimulationMesh::is_point_in_triangle(
    const Vector3r& p,
    const Vector3r& a,
    const Vector3r& b,
    const Vector3r& c)
{
    Vector3r AB_AC = (b - a).cross(c - a);
    Vector3r AB_AP = (b - a).cross(p - a);
    Vector3r BC_BP = (c - b).cross(p - b);
    Vector3r CA_CP = (a - c).cross(p - c);
    if (AB_AC.dot(AB_AP) < 0 || AB_AC.dot(BC_BP) < 0 || AB_AC.dot(CA_CP) < 0) return false;
    return true;
}

bool ImageSimulationMesh::is_point_in_collection(const Vector3r& p, size_t collection_id)
{
    for (int i = 0;
         i < triangle_collections_from_input_surface.collections[collection_id].face_ids.size();
         i++) {
        size_t tri_vid1 =
            triangle_collections_from_input_surface.input_faces
                [triangle_collections_from_input_surface.collections[collection_id].face_ids[i]][0];
        size_t tri_vid2 =
            triangle_collections_from_input_surface.input_faces
                [triangle_collections_from_input_surface.collections[collection_id].face_ids[i]][1];
        size_t tri_vid3 =
            triangle_collections_from_input_surface.input_faces
                [triangle_collections_from_input_surface.collections[collection_id].face_ids[i]][2];
        Vector3r tri_v1_pos =
            triangle_collections_from_input_surface.input_vertices_rational[tri_vid1];
        Vector3r tri_v2_pos =
            triangle_collections_from_input_surface.input_vertices_rational[tri_vid2];
        Vector3r tri_v3_pos =
            triangle_collections_from_input_surface.input_vertices_rational[tri_vid3];
        if (is_point_in_triangle(p, tri_v1_pos, tri_v2_pos, tri_v3_pos)) return true;
    }
    return false;
}

int ImageSimulationMesh::find_collection_for_tracked_surface(const Tuple& t)
{
    size_t fid = t.fid(*this);
    size_t vid1 = t.vid(*this);
    size_t vid2 = t.switch_vertex(*this).vid(*this);
    size_t vid3 = t.switch_edge(*this).switch_vertex(*this).vid(*this);
    int in_collection = -1;

    bool flag = false;

    Vector3r v1_pos = m_vertex_attribute[vid1].m_pos;
    Vector3r v2_pos = m_vertex_attribute[vid2].m_pos;
    Vector3r v3_pos = m_vertex_attribute[vid3].m_pos;

    for (size_t i = 0; i < triangle_collections_from_input_surface.collections.size(); i++) {
        // check coplanarity
        if (!is_triangle_coplanar_collection(
                v1_pos,
                v2_pos,
                v3_pos,
                triangle_collections_from_input_surface.collections[i]))
            continue;

        // debug code
        // in_collection = i;
        // break;

        // check containment by point in any triangle in the collection
        if (is_point_in_collection(v1_pos, i) && is_point_in_collection(v2_pos, i) &&
            is_point_in_collection(v3_pos, i)) {
            in_collection = i;
            break;
        }
    }

    // debug code
    // if (in_collection == -1) {
    //     int o1 = orient3d_t(
    //         triangle_collections_from_input_surface.collections[1].a_pos,
    //         triangle_collections_from_input_surface.collections[1].b_pos,
    //         triangle_collections_from_input_surface.collections[1].c_pos,
    //         v1_pos);

    //     int o2 = orient3d_t(
    //         triangle_collections_from_input_surface.collections[1].a_pos,
    //         triangle_collections_from_input_surface.collections[1].b_pos,
    //         triangle_collections_from_input_surface.collections[1].c_pos,
    //         v2_pos);

    //     int o3 = orient3d_t(
    //         triangle_collections_from_input_surface.collections[1].a_pos,
    //         triangle_collections_from_input_surface.collections[1].b_pos,
    //         triangle_collections_from_input_surface.collections[1].c_pos,
    //         v3_pos);

    //     std::cout.precision(15);
    //     std::cout << "o1: " << o1 << std::endl;
    //     std::cout << "o2: " << o2 << std::endl;
    //     std::cout << "o3: " << o3 << std::endl;

    //     std::cout << "a_pos: " << triangle_collections_from_input_surface.collections[1].a_pos[0]
    //               << " " << triangle_collections_from_input_surface.collections[1].a_pos[1] << "
    //               "
    //               << triangle_collections_from_input_surface.collections[1].a_pos[2] <<
    //               std::endl;
    //     std::cout << "b_pos: " << triangle_collections_from_input_surface.collections[1].b_pos[0]
    //               << " " << triangle_collections_from_input_surface.collections[1].b_pos[1] << "
    //               "
    //               << triangle_collections_from_input_surface.collections[1].b_pos[2] <<
    //               std::endl;
    //     std::cout << "c_pos: " << triangle_collections_from_input_surface.collections[1].c_pos[0]
    //               << " " << triangle_collections_from_input_surface.collections[1].c_pos[1] << "
    //               "
    //               << triangle_collections_from_input_surface.collections[1].c_pos[2] <<
    //               std::endl;
    //     std::cout << "v1_pos: " << v1_pos[0] << " " << v1_pos[1] << " " << v1_pos[2] <<
    //     std::endl; std::cout << "v2_pos: " << v2_pos[0] << " " << v2_pos[1] << " " << v2_pos[2]
    //     << std::endl; std::cout << "v3_pos: " << v3_pos[0] << " " << v3_pos[1] << " " <<
    //     v3_pos[2] << std::endl;

    //     std::ofstream outfile("unmatched_triangle.obj");
    //     outfile << "v " << v1_pos[0] << " " << v1_pos[1] << " " << v1_pos[2] << std::endl;
    //     outfile << "v " << v2_pos[0] << " " << v2_pos[1] << " " << v2_pos[2] << std::endl;
    //     outfile << "v " << v3_pos[0] << " " << v3_pos[1] << " " << v3_pos[2] << std::endl;
    //     outfile << "f 1 2 3";
    //     exit(0);
    // }

    return in_collection;
}

bool ImageSimulationMesh::check_vertex_param_type()
{
    // std::ofstream file("missing_param_v.obj");
    bool flag = true;
    return flag;
}

int ImageSimulationMesh::flood_fill()
{
    int current_id = 0;
    auto tets = get_tets();
    std::map<size_t, bool> visited;

    for (auto t : tets) {
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

        std::cout << std::endl;

        current_id++;
    }
    return current_id;
}

void ImageSimulationMesh::save_paraview(const std::string& path, const bool use_hdf5)
{
    consolidate_mesh();
    // flood fill
    int num_parts = flood_fill();
    std::cout << "flood fill parts: " << num_parts << std::endl;
    const auto& vs = get_vertices();
    const auto& tets = get_tets();

    Eigen::MatrixXd V(vs.size(), 3);
    Eigen::MatrixXi T(tets.size(), 4);

    Eigen::MatrixXd parts(tets.size(), 1);
    Eigen::MatrixXd wn(tets.size(), 1);

    int index = 0;
    for (auto t : tets) {
        size_t tid = t.tid(*this);
        parts(index, 0) = m_tet_attribute[tid].part_id;
        wn(index, 0) = m_tet_attribute[tid].m_winding_number;

        const auto& vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            T(index, j) = vs[j].vid(*this);
        }
        ++index;
    }

    for (auto v : vs) {
        const auto vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
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
    writer->add_cell_field("winding_number", wn);
    writer->write_mesh(out_path, V, T);
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