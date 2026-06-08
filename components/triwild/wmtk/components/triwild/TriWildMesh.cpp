
#include "TriWildMesh.h"

#include "wmtk/utils/Rational.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/AMIPS2D.h>
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
#include <igl/read_triangle_mesh.h>
#include <igl/Timer.h>
#include <igl/orientable_patches.h>
#include <wmtk/utils/EnableWarnings.hpp>
#include <wmtk/utils/GeoUtils.h>
// clang-format on

//#include <paraviewo/HDF5VTUWriter.hpp>
#include <bitset>
#include <limits>
#include <paraviewo/VTUWriter.hpp>

namespace wmtk::components::triwild {

VertexAttributes::VertexAttributes(const Vector2r& p)
{
    m_pos = p;
    m_posf = to_double(p);
}

void TriWildMesh::mesh_improvement(int max_its)
{
    ////preprocessing
    // TODO: refactor to eliminate repeated partition.
    //

    log_and_throw_error("TODO");
    // compute_vertex_partition_morton();

    // save_paraview(fmt::format("debug_{}", debug_print_counter++), false);

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
        wmtk::logger().info("max energy {:.6} | stop {:.6}", max_energy, m_params.stop_energy);
        if (max_energy < m_params.stop_energy) {
            break;
        }
        consolidate_mesh();

        wmtk::logger().info("#V = {}, #T = {}", vert_capacity(), tri_capacity());

        auto cnt_round = 0, cnt_verts = 0;
        TriMesh::for_each_vertex([&](auto& v) {
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

std::tuple<double, double> TriWildMesh::local_operations(
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
        const auto faces = get_edges_by_condition([](auto& f) { return f.m_is_surface_fs; });
        for (const auto& verts : faces) {
            const auto& p0 = m_vertex_attribute[verts[0]].m_posf;
            const auto& p1 = m_vertex_attribute[verts[1]].m_posf;
            if (m_envelope->is_outside(std::array<Vector2d, 2>{p0, p1})) {
                logger().error("Edge {} is outside!", verts);
            }
        }

        // check for inverted faces
        for (const Tuple& t : get_faces()) {
            if (!is_inverted(t)) {
                continue;
            }
            const auto vs = oriented_tri_vids(t);
            logger().error("Face {} is inverted! Vertices = {}", t.fid(*this), vs);
        }
        logger().info("Sanity checks done.");
    };

    sanity_checks();

    timer.start();
    for (int i = 0; i < ops.size(); i++) {
        if (i == 0) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==splitting {}==", n);
                split_all_edges();
                wmtk::logger().info(
                    "#V = {}, #F = {} after split",
                    get_vertices().size(),
                    get_faces().size());
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("split max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 1) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==collapsing {}==", n);
                collapse_all_edges(collapse_limit_length);
                wmtk::logger().info(
                    "#V = {}, #F = {} after collapse",
                    get_vertices().size(),
                    get_faces().size());
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("collapse max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 2) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==swapping {}==", n);
                size_t cnt_success = swap_all_edges();
                if (cnt_success == 0) {
                    break;
                }
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("swap max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 3) {
            wmtk::logger().info("==smoothing ==");
            smooth_all_vertices(ops[i]);
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("smooth max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        }
    }
    energy = get_max_avg_energy();
    logger().info("max energy = {:.6}", std::get<0>(energy));
    logger().info("avg energy = {:.6}", std::get<1>(energy));
    logger().info("time = {:.4}s", timer.getElapsedTimeInSec());


    return energy;
}

bool TriWildMesh::adjust_sizing_field_serial(double max_energy)
{
    wmtk::logger().info("#V {}, #F {}", vert_capacity(), tri_capacity());

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

    for (int i = 0; i < tri_capacity(); i++) {
        const Tuple t = tuple_from_tri(i);
        if (!t.is_valid(*this)) {
            continue;
        }
        const size_t fid = t.fid(*this);
        if (m_face_attribute.at(fid).m_quality < filter_energy) {
            continue;
        }
        const auto vs = oriented_tri_vids(t);
        Vector2d c(0, 0); // center
        for (int j = 0; j < 3; j++) {
            c += m_vertex_attribute.at(vs[j]).m_posf;
            v_queue.emplace(vs[j]);
        }
        c /= 3;
        pts.emplace_back(Vector3d(c[0], c[1], 0));
    }

    wmtk::logger().info("filter energy {} Low Quality Tets {}", filter_energy, pts.size());

    const double R = m_params.l * 1.8;

    int sum = 0;
    int adjcnt = 0;

    std::vector<bool> visited(vert_capacity(), false);

    KNN knn(pts);

    std::vector<size_t> cache_one_ring;
    // size_t vid;
    while (!v_queue.empty()) {
        sum++;
        const size_t vid = v_queue.front();
        v_queue.pop();
        if (visited[vid]) continue;
        visited[vid] = true;
        adjcnt++;

        const Vector2d& pos_v = m_vertex_attribute.at(vid).m_posf;
        const Vector3d p(pos_v[0], pos_v[1], 0);
        double sq_dist = 0.;
        uint32_t idx;
        knn.nearest_neighbor(p, idx, sq_dist);
        const double dist = std::sqrt(sq_dist);

        if (dist > R) { // outside R-ball, unmark.
            continue;
        }

        scale_multipliers[vid] = std::min(
            scale_multipliers[vid],
            dist / R * (1 - refine_scalar) + refine_scalar); // linear interpolate

        get_one_ring_vids_for_vertex_duplicate(vid, cache_one_ring);
        for (size_t n_vid : cache_one_ring) {
            if (visited[n_vid]) {
                continue;
            }
            v_queue.push(n_vid);
        }
    }

    logger().info("sum = {}; adjacent = {}", sum, adjcnt);

    std::atomic_bool is_hit_min_edge_length = false;

    for (int i = 0; i < vert_capacity(); i++) {
        const Tuple v = tuple_from_vertex(i);
        if (!v.is_valid(*this)) {
            continue;
        }
        const size_t vid = v.vid(*this);
        auto& v_attr = m_vertex_attribute[vid];

        auto new_scale = v_attr.m_sizing_scalar * scale_multipliers[vid];
        if (new_scale > 1) {
            v_attr.m_sizing_scalar = 1;
        } else if (new_scale < min_refine_scalar) {
            is_hit_min_edge_length = true;
            v_attr.m_sizing_scalar = min_refine_scalar;
        } else {
            v_attr.m_sizing_scalar = new_scale;
        }
    }

    return is_hit_min_edge_length.load();
}

void TriWildMesh::compute_winding_number(
    const std::vector<Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces)
{
    log_and_throw_error("winding number not implemented yet");
    // Eigen::MatrixXd V;
    // Eigen::MatrixXi F;
    // if (!vertices.empty()) {
    //     V.resize(vertices.size(), 3);
    //     F.resize(faces.size(), 3);

    //     for (size_t i = 0; i < (size_t)V.rows(); i++) {
    //         V.row(i) = vertices[i];
    //     }
    //     for (size_t i = 0; i < (size_t)F.rows(); i++) {
    //         for (size_t j = 0; j < 3; j++) {
    //             F(i, j) = (int)faces[i][j];
    //         }
    //     }
    // } else { // use track to filter
    //     auto outface = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });
    //     V = Eigen::MatrixXd::Zero(vert_capacity(), 3);
    //     for (auto v : get_vertices()) {
    //         auto vid = v.vid(*this);
    //         V.row(vid) = m_vertex_attribute[vid].m_posf;
    //     }
    //     F.resize(outface.size(), 3);
    //     for (auto i = 0; i < outface.size(); i++) {
    //         F.row(i) << (int)outface[i][0], (int)outface[i][1], (int)outface[i][2];
    //     }
    //     // wmtk::logger().info("Output face size {}", outface.size());
    //     auto F0 = F;
    //     Eigen::VectorXi C;
    //     bfs_orient(F0, F, C);
    //     // wmtk::logger().info("BFS orient {}", F.rows());
    // }

    // const auto& tets = get_tets();
    // Eigen::MatrixXd C = Eigen::MatrixXd::Zero(tets.size(), 3);
    // for (size_t i = 0; i < tets.size(); i++) {
    //     auto vs = oriented_tet_vertices(tets[i]);
    //     for (auto& v : vs) C.row(i) += m_vertex_attribute[v.vid(*this)].m_posf;
    //     C.row(i) /= 4;
    // }

    // Eigen::VectorXd W;
    // igl::winding_number(V, F, C, W);

    // if (W.maxCoeff() <= 0.5) {
    //     // all removed, let's invert.
    //     wmtk::logger().info("Correcting winding number");
    //     for (auto i = 0; i < F.rows(); i++) {
    //         auto temp = F(i, 0);
    //         F(i, 0) = F(i, 1);
    //         F(i, 1) = temp;
    //     }
    //     igl::winding_number(V, F, C, W);
    // }

    // if (W.maxCoeff() <= 0.5) {
    //     wmtk::logger().critical("Still Inverting..., Empty Output");
    //     return;
    // }

    // // store winding number in mesh
    // if (vertices.empty()) {
    //     // from tracked surface
    //     for (int i = 0; i < tets.size(); ++i) {
    //         const size_t tid = tets[i].tid(*this);
    //         m_tet_attribute[tid].m_winding_number_tracked = W(i);
    //     }
    // } else {
    //     // from input surface
    //     for (int i = 0; i < tets.size(); ++i) {
    //         const size_t tid = tets[i].tid(*this);
    //         m_tet_attribute[tid].m_winding_number_input = W(i);
    //     }
    // }
}

void TriWildMesh::compute_winding_numbers(const std::vector<std::string>& input_paths)
{
    log_and_throw_error("winding number not implemented yet");

    // const auto& tets = get_tets();
    // Eigen::MatrixXd C = Eigen::MatrixXd::Zero(tets.size(), 3);
    // for (size_t i = 0; i < tets.size(); i++) {
    //     const auto vs = oriented_tet_vids(tets[i]);
    //     for (size_t v : vs) {
    //         C.row(i) += m_vertex_attribute[v].m_posf;
    //     }
    //     C.row(i) /= 4;
    // }

    // for (const std::string& input_path : input_paths) {
    //     MatrixXd inV, V;
    //     MatrixXi inF, F;
    //     igl::read_triangle_mesh(input_path, inV, inF);
    //     VectorXi _I;
    //     igl::remove_unreferenced(inV, inF, V, F, _I);
    //     assert(V.cols() == 3);
    //     assert(F.cols() == 3);

    //     // compute winding number for V,F
    //     Eigen::VectorXd W;
    //     igl::winding_number(V, F, C, W);

    //     if (W.maxCoeff() <= 0.5) {
    //         // all removed, let's invert.
    //         wmtk::logger().info("Correcting winding number");
    //         for (auto i = 0; i < F.rows(); i++) {
    //             auto temp = F(i, 0);
    //             F(i, 0) = F(i, 1);
    //             F(i, 1) = temp;
    //         }
    //         igl::winding_number(V, F, C, W);
    //     }

    //     if (W.maxCoeff() <= 0.5) {
    //         wmtk::logger().warn("No winding number above 0.5 for input_path {}", input_path);
    //     }

    //     // store winding number in mesh
    //     for (int i = 0; i < tets.size(); ++i) {
    //         const size_t tid = tets[i].tid(*this);
    //         m_tet_attribute[tid].m_winding_number_per_input.push_back(W(i));
    //     }
    // }
}

void TriWildMesh::filter_with_input_surface_winding_number()
{
    log_and_throw_error("winding number not implemented yet");
    // std::vector<size_t> rm_tids;
    // for (const Tuple& t : get_tets()) {
    //     const size_t tid = t.tid(*this);
    //     if (m_tet_attribute[tid].m_winding_number_input <= 0.5) {
    //         rm_tids.emplace_back(tid);
    //     }
    // }

    // remove_tets_by_ids(rm_tids);
}

void TriWildMesh::filter_with_tracked_surface_winding_number()
{
    log_and_throw_error("winding number not implemented yet");
    // std::vector<size_t> rm_tids;
    // for (const Tuple& t : get_tets()) {
    //     const size_t tid = t.tid(*this);
    //     if (m_tet_attribute[tid].m_winding_number_tracked <= 0.5) {
    //         rm_tids.emplace_back(tid);
    //     }
    // }

    // remove_tets_by_ids(rm_tids);
}

void TriWildMesh::filter_with_flood_fill()
{
    log_and_throw_error("flood fill filter not implemented yet");
    // std::map<int, size_t> id_counter;

    // // find ID that appears the most on the boundary
    // for (const Tuple& t : get_faces()) {
    //     if (t.switch_tetrahedron(*this)) {
    //         // face is interior
    //         continue;
    //     }
    //     // face is boundary
    //     const int id = m_tet_attribute[t.tid(*this)].part_id;

    //     if (id_counter.count(id) == 0) {
    //         id_counter[id] = 1;
    //     } else {
    //         id_counter[id]++;
    //     }
    // }

    // if (id_counter.size() != 1) {
    //     logger().warn(
    //         "There were {} flood fill IDs found at the boundary. Using the one with most "
    //         "occurances.",
    //         id_counter.size());
    // }

    // int best_id = id_counter.begin()->first;
    // size_t best_count = id_counter.begin()->second;
    // for (const auto& [id, count] : id_counter) {
    //     if (count > best_count) {
    //         best_id = id;
    //         best_count = count;
    //     }
    // }

    // logger().info("Filter with flood fill ID {}", best_id);

    // std::vector<size_t> rm_tids;
    // for (const Tuple& t : get_tets()) {
    //     const size_t tid = t.tid(*this);
    //     if (m_tet_attribute[tid].part_id == best_id) {
    //         rm_tids.emplace_back(tid);
    //     }
    // }

    // remove_tets_by_ids(rm_tids);
}

double TriWildMesh::get_length2(const Tuple& l) const
{
    auto& m = *this;
    auto& v1 = l;
    auto v2 = l.switch_vertex(m);
    double length =
        (m.m_vertex_attribute[v1.vid(m)].m_posf - m.m_vertex_attribute[v2.vid(m)].m_posf)
            .squaredNorm();
    return length;
}

std::tuple<double, double> TriWildMesh::get_max_avg_energy()
{
    double max_energy = -1.;
    double avg_energy = 0.;
    auto cnt = 0;

    for (int i = 0; i < tri_capacity(); i++) {
        const Tuple tup = tuple_from_tri(i);
        if (!tup.is_valid(*this)) {
            continue;
        }
        const double q = m_face_attribute[tup.fid(*this)].m_quality;
        max_energy = std::max(max_energy, q);
        avg_energy += q;
        cnt++;
    }

    avg_energy /= cnt;

    return std::make_tuple(max_energy, avg_energy);
}


bool TriWildMesh::is_inverted_f(const Tuple& loc) const
{
    auto vs = oriented_tri_vids(loc);

    igl::predicates::exactinit();
    auto res = igl::predicates::orient2d(
        m_vertex_attribute[vs[0]].m_posf,
        m_vertex_attribute[vs[1]].m_posf,
        m_vertex_attribute[vs[2]].m_posf);
    if (res == igl::predicates::Orientation::POSITIVE) {
        return false;
    }
    return true;
}

bool TriWildMesh::is_inverted(const std::array<size_t, 3>& vs) const
{
    // Return a positive value if the point pd lies below the
    // plane passing through pa, pb, and pc; "below" is defined so
    // that pa, pb, and pc appear in counterclockwise order when
    // viewed from above the plane.

    if (m_vertex_attribute[vs[0]].m_is_rounded && m_vertex_attribute[vs[1]].m_is_rounded &&
        m_vertex_attribute[vs[2]].m_is_rounded) {
        igl::predicates::exactinit();
        auto res = igl::predicates::orient2d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf);
        if (res == igl::predicates::Orientation::POSITIVE) {
            return false;
        }
        return true;
    } else {
        log_and_throw_error("TODO: non-rounded orientation test");
        // Vector3r n =
        //     ((m_vertex_attribute[vs[1]].m_pos) - m_vertex_attribute[vs[0]].m_pos)
        //         .cross((m_vertex_attribute[vs[2]].m_pos) - m_vertex_attribute[vs[0]].m_pos);
        // Vector3r d = (m_vertex_attribute[vs[3]].m_pos) - m_vertex_attribute[vs[0]].m_pos;
        // auto res = n.dot(d);
        // if (res > 0) // predicates returns pos value: non-inverted
        //     return false;
        // else
        //     return true;
    }
}

bool TriWildMesh::is_inverted(const Tuple& loc) const
{
    auto vs = oriented_tri_vids(loc);
    return is_inverted(vs);
}

bool TriWildMesh::round(const Tuple& v)
{
    size_t i = v.vid(*this);
    if (m_vertex_attribute[i].m_is_rounded) {
        return true;
    }

    auto old_pos = m_vertex_attribute[i].m_pos;
    m_vertex_attribute[i].m_pos << m_vertex_attribute[i].m_posf[0], m_vertex_attribute[i].m_posf[1];
    auto conn_tets = get_one_ring_tris_for_vertex(v);
    m_vertex_attribute[i].m_is_rounded = true;
    for (const Tuple& tet : conn_tets) {
        if (is_inverted(tet)) {
            m_vertex_attribute[i].m_is_rounded = false;
            m_vertex_attribute[i].m_pos = old_pos;
            return false;
        }
    }

    return true;
}

double TriWildMesh::get_quality(const std::array<size_t, 3>& vs) const
{
    std::array<Vector2d, 3> ps;
    for (size_t k = 0; k < 3; k++) {
        ps[k] = m_vertex_attribute[vs[k]].m_posf;
    }
    double energy = -1.;
    {
        std::array<double, 6> T;
        for (size_t k = 0; k < 3; k++)
            for (size_t j = 0; j < 2; j++) {
                T[k * 2 + j] = ps[k][j];
            }
        energy = AMIPS2D_energy(T);
    }
    if (std::isinf(energy) || std::isnan(energy) || energy < 2 - 1e-3) {
        return MAX_ENERGY;
    }
    return energy;
}

double TriWildMesh::get_quality(const Tuple& loc) const
{
    auto its = oriented_tri_vids(loc);
    return get_quality(its);
}

std::vector<std::array<size_t, 2>> TriWildMesh::get_edges_by_condition(
    std::function<bool(const EdgeAttributes&)> cond) const
{
    std::vector<std::array<size_t, 2>> res;
    for (const Tuple& e : get_edges()) {
        size_t eid = e.eid(*this);
        if (cond(m_edge_attribute[eid])) {
            res.push_back({{e.vid(*this), e.switch_vertex(*this).vid(*this)}});
        }
    }
    return res;
}

bool TriWildMesh::is_edge_on_surface(const Tuple& loc) const
{
    const auto vs = get_edge_vids(loc);
    if (!m_vertex_attribute.at(vs[0]).m_is_on_surface ||
        !m_vertex_attribute.at(vs[1]).m_is_on_surface) {
        return false;
    }

    const size_t eid = loc.eid(*this);
    return m_edge_attribute[eid].m_is_surface_fs;
}
bool TriWildMesh::is_edge_on_surface(const std::array<size_t, 2>& vids) const
{
    if (!m_vertex_attribute.at(vids[0]).m_is_on_surface ||
        !m_vertex_attribute.at(vids[1]).m_is_on_surface) {
        return false;
    }

    const auto [_, eid] = tuple_from_edge(vids);
    return m_edge_attribute[eid].m_is_surface_fs;
}
bool TriWildMesh::is_edge_on_bbox(const Tuple& loc) const
{
    const auto vs = get_edge_vids(loc);
    if (m_vertex_attribute.at(vs[0]).on_bbox_faces.empty() ||
        m_vertex_attribute.at(vs[1]).on_bbox_faces.empty()) {
        return false;
    }

    const size_t eid = loc.eid(*this);
    return m_edge_attribute[eid].m_is_bbox_fs >= 0;
}

bool TriWildMesh::is_edge_on_bbox(const std::array<size_t, 2>& vids) const
{
    if (m_vertex_attribute.at(vids[0]).on_bbox_faces.empty() ||
        m_vertex_attribute.at(vids[1]).on_bbox_faces.empty()) {
        return false;
    }
    const auto [_, eid] = tuple_from_edge(vids);
    return m_edge_attribute[eid].m_is_bbox_fs >= 0;
}

bool TriWildMesh::is_vertex_on_boundary(const size_t e0)
{
    log_and_throw_error("TODO: implement is_vertex_on_boundary");
    // if (!m_vertex_attribute.at(e0).m_is_on_open_boundary) {
    //     return false;
    // }

    // const auto neigh_vids = get_one_ring_vids_for_vertex(e0);
    // const auto e0_tids = get_one_ring_tids_for_vertex(e0);

    // for (const size_t e1 : neigh_vids) {
    //     if (!m_vertex_attribute.at(e1).m_is_on_open_boundary) {
    //         continue;
    //     }
    //     int cnt = 0;
    //     for (size_t t_id : e0_tids) {
    //         const auto vs = oriented_tet_vids(t_id);
    //         std::array<int, 4> opp_js; // DZ: all vertices that are adjacent to e1 except for e2
    //         int ii = 0;
    //         for (int j = 0; j < 4; j++) {
    //             if (vs[j] == e0 || vs[j] == e1) {
    //                 continue;
    //             }
    //             opp_js[ii++] = j;
    //         }
    //         // DZ: if the tet contains e1 and e2, then ii == 2
    //         if (ii != 2) {
    //             continue;
    //         }
    //         // DZ: opp_js vertices form a tet together with v1,v2
    //         if (m_vertex_attribute.at(vs[opp_js[0]]).m_is_on_surface) {
    //             const auto [f0_tup, f0_id] = tuple_from_face({{e0, e1, vs[opp_js[0]]}});
    //             if (m_face_attribute.at(f0_id).m_is_surface_fs) {
    //                 cnt++;
    //             }
    //         }
    //         if (m_vertex_attribute.at(vs[opp_js[1]]).m_is_on_surface) {
    //             const auto [f1_tup, f1_id] = tuple_from_face({{e0, e1, vs[opp_js[1]]}});
    //             if (m_face_attribute.at(f1_id).m_is_surface_fs) {
    //                 cnt++;
    //             }
    //         }
    //         if (cnt > 2) {
    //             break;
    //         }
    //     }
    //     // all faces are visited twice, so cnt == 2 means there is one boundary face
    //     if (cnt == 2) {
    //         // this is a boundary edge
    //         return true;
    //     }
    // }

    // return false;
}

} // namespace wmtk::components::triwild
