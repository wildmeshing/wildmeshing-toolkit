#include <wmtk/TetOptimizerMesh.h>

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/GeoUtils.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/RunPass.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>
#include <wmtk/utils/partition_utils.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/predicates.hpp>
#include <igl/write_triangle_mesh.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <cstdlib>
#include <queue>

namespace wmtk {

std::tuple<double, double> TetOptimizerMesh::optimization_quality_stats()
{
    return get_max_avg_energy();
}

void TetOptimizerMesh::mesh_improvement(int max_its)
{
    m_iterations_used = 0;
    if (optimization_stop_at_float() && round_and_check_all_rounded()) {
        logger().info("===== All vertices are rounded. Stop. =====");
        return;
    }

    compute_vertex_partition_morton();

    logger().info("========it pre========");
    // Performing swaps after the initial collapse improves quality and convergence.
    local_operations({{0, 1, 1, 0}}, false);

    double pre_max_metric = std::get<0>(optimization_quality_stats());
    logger().info("max energy {:.6} | stop {:.6}", pre_max_metric, optimization_stop_metric());
    int refine_cooldown = m_params.stuck_refine_cooldown;

    for (int it = 0; it < max_its; ++it) {
        m_iterations_used = it + 1;
        logger().info("\n========it {}========", it);

        double max_metric = 0.;
        double avg_metric = 0.;
        if (!m_params.interleaved_smoothing) {
            std::tie(max_metric, avg_metric) =
                local_operations({{1, 1, 1, m_params.num_smoothing_passes}});
        } else {
            const int k = m_params.interleaved_smoothing_passes;
            const std::array<std::array<int, 4>, 3> passes = {
                {{{1, 0, 0, k}}, {{0, 1, 0, k}}, {{0, 0, 1, k}}}};
            for (const auto& ops : passes) {
                std::tie(max_metric, avg_metric) = local_operations(ops);
                if (max_metric < optimization_stop_metric()) break;
            }
        }

        logger().info("max energy {:.6} | stop {:.6}", max_metric, optimization_stop_metric());

        std::atomic<int> n_round = 0;
        std::atomic<int> n_verts = 0;
        TetMesh::for_each_vertex([&](auto& v) {
            if (m_vertex_attribute[v.vid(*this)].m_is_rounded) {
                n_round.fetch_add(1, std::memory_order_relaxed);
            }
            n_verts.fetch_add(1, std::memory_order_relaxed);
        });
        const int cnt_round = n_round.load(std::memory_order_relaxed);
        const int cnt_verts = n_verts.load(std::memory_order_relaxed);
        if (cnt_round < cnt_verts) {
            logger().info("rounded {}/{}", cnt_round, cnt_verts);
        } else {
            logger().info("All rounded!");
        }

        if (max_metric < optimization_stop_metric()) {
            if (cnt_round == cnt_verts) break;
            logger().info(
                "energy target reached, but {} of {} vertices are still un-rounded; continuing",
                cnt_verts - cnt_round,
                cnt_verts);
        }

        consolidate_mesh();
        logger().info("#V = {}, #T = {}", vert_capacity(), tet_capacity());

        if (optimization_stop_at_float() && round_and_check_all_rounded()) {
            logger().info("All vertices are rounded. Stop.");
            break;
        }

        if (refine_cooldown > 0) {
            --refine_cooldown;
        } else if (
            it > 0 && max_metric > optimization_stop_metric() &&
            (pre_max_metric - max_metric) <=
                m_params.stuck_refine_stall_eps * (max_metric - optimization_stop_metric())) {
            logger().info(">>>>stuck-refine (maxE {:.6} stalled)...", max_metric);
            refine_sizing_around_worst(max_metric);
            logger().info(">>>>stuck-refine finished...");
            refine_cooldown = m_params.stuck_refine_cooldown;
        }
        pre_max_metric = max_metric;
    }

    logger().info("========it post========");
    local_operations({{0, 1, 0, 0}});

    // Removing what the mesh does not need is the last thing to do, not something to
    // interleave: it trades cells for nothing but the guarantee that the max energy does not
    // rise, which is only worth taking once the energy is where it is going to end up.
    coarsen_mesh();
}

std::tuple<double, double> TetOptimizerMesh::local_operations(
    const std::array<int, 4>& ops,
    bool collapse_limit_length)
{
    igl::Timer timer;

    static constexpr std::array<const char*, 4> names = {{"split", "collapse", "swap", "smooth"}};

    auto sanity_checks = [this]() {
        if (!m_params.perform_sanity_checks) return;

        logger().info("Perform sanity checks...");
        const auto faces = get_faces_by_condition([](const auto& f) { return f.m_is_surface_fs; });
        for (const auto& verts : faces) {
            if (surface_triangle_is_outside(verts[0], verts[1], verts[2])) {
                logger().error("Face {} is outside!", verts);
            }
        }
        for (const Tuple& t : get_tets()) {
            if (is_inverted(t)) {
                logger().error(
                    "Tet {} is inverted! Vertices = {}",
                    t.tid(*this),
                    oriented_tet_vids(t));
            }
        }
        optimization_sanity_checks_extra();
        logger().info("Sanity checks done.");
    };

    sanity_checks();
    update_attributes();
    size_t retry_count = 0;
    for (int i = 0; i < int(ops.size()); ++i) {
        if (retry_count > 0) {
            logger().info(
                "Retrying {} pass after consolidating. Retry count: {}",
                names[size_t(i)],
                retry_count);
        }
        timer.start();
        if (i == 0) {
            for (int n = 0; n < ops[i]; ++n) {
                logger().info("==splitting {}==", n);
                split_all_edges();
                logger().info(
                    "#V = {}, #T = {} after split",
                    get_vertices().size(),
                    get_tets().size());
            }
        } else if (i == 1) {
            for (int n = 0; n < ops[i]; ++n) {
                logger().info("==collapsing {}==", n);
                collapse_all_edges(collapse_limit_length);
                logger().info(
                    "#V = {}, #T = {} after collapse",
                    get_vertices().size(),
                    get_tets().size());
            }
        } else if (i == 2) {
            for (int n = 0; n < ops[i]; ++n) {
                logger().info("==swapping {}==", n);
                const size_t cnt_success = swap_all_edges_all() + swap_all_faces();
                if (cnt_success == 0) break;
            }
        } else {
            smooth_all_vertices(size_t(ops[i]));
            if (ops[i] > 0) round_all_vertices();
        }

        if (ops[i] > 0) {
            if (m_params.debug_output && i < 3) {
                // no need to print debug output for smoothing, since it prints already internally
                write_optimization_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
            }
            const auto [max_metric, avg_metric] = optimization_quality_stats();
            logger().info(
                "{} max energy = {:.6} avg = {:.6}",
                names[size_t(i)],
                max_metric,
                avg_metric);
            if (i == 2) {
                logger().info(
                    "cnt_surface_swap (cumulative) = {} [3-2: {}, 4-4: {}, 5-6: {}]",
                    cnt_surface_swap.load(),
                    cnt_surface_swap_32.load(),
                    cnt_surface_swap_44.load(),
                    cnt_surface_swap_56.load());
            }
            sanity_checks();
            update_attributes();
        }

        // A pass that ran out of preallocated slots abandoned operations for want of storage.
        // Consolidating both reclaims the slots removed elements still hold -- the counter only
        // advances during a pass, so churn alone can exhaust it -- and re-derives the storage
        // from the real element count, which grows it by m_preallocation_factor when there is
        // no churn left to reclaim.
        //
        // Consolidate renumbers, so the abandoned operations' tuples are gone; the group is
        // re-run instead, which re-collects them against the enlarged storage. Every retry
        // grows the storage by the factor, so the chain makes progress and terminates.
        if (slots_exhausted()) {
            const size_t live_before = tet_capacity();
            const size_t store_before = tet_storage_capacity();
            consolidate_mesh();
            clear_slots_exhausted();
            logger().info(
                "{} pass exhausted its preallocated slots: {} of {} tets reclaimed as "
                "churn, storage {} -> {}",
                names[size_t(i)],
                live_before - tet_capacity(),
                live_before,
                store_before,
                tet_storage_capacity());
            --i; // retry the same operation after consolidating
            ++retry_count;
        } else {
            retry_count = 0;
        }
    }

    const auto stats = optimization_quality_stats();
    logger().info("max energy = {:.6}", std::get<0>(stats));
    logger().info("avg energy = {:.6}", std::get<1>(stats));
    logger().info("time = {:.4}s", timer.getElapsedTime());
    return stats;
}

TetOptimizerMesh::VertexAttributes::VertexAttributes(const Vector3r& p)
{
    m_pos = p;
    m_posf = to_double(m_pos);
}

void TetOptimizerMesh::compute_vertex_partition()
{
    auto partition_id = partition_TetMesh(*this, NUM_THREADS);
    for (auto i = 0; i < vert_capacity(); i++) m_vertex_attribute[i].partition_id = partition_id[i];
}

void TetOptimizerMesh::compute_vertex_partition_morton()
{
    if (NUM_THREADS == 0) {
        return;
    }

    logger().info("Number of parts: {} by morton", NUM_THREADS);

    std::vector<size_t> partition_id;
    wmtk::partition_vertex_morton(
        vert_capacity(),
        [this](size_t i) { return m_vertex_attribute[i].m_posf; },
        NUM_THREADS,
        partition_id);

    for (size_t i = 0; i < partition_id.size(); i++) {
        m_vertex_attribute[i].partition_id = partition_id[i];
    }
}

double TetOptimizerMesh::swap_edge_44_energy(
    const std::vector<std::array<size_t, 4>>& tets,
    const int op_case)
{
    double max_energy = -1;
    for (const auto& vids : tets) {
        if (is_inverted(vids)) {
            return std::numeric_limits<double>::max();
        }
        const double e = get_quality(vids);
        max_energy = std::max(max_energy, e);
    }
    return max_energy;
}

double TetOptimizerMesh::swap_edge_56_energy(
    const std::vector<std::array<size_t, 4>>& tets,
    const int op_case)
{
    double max_energy = -1;
    for (const auto& vids : tets) {
        if (is_inverted(vids)) {
            return std::numeric_limits<double>::max();
        }
        const double e = get_quality(vids);
        max_energy = std::max(max_energy, e);
    }
    return max_energy;
}

bool TetOptimizerMesh::smooth_before(const Tuple& t)
{
    const bool r = round(t);

    const size_t vid = t.vid(*this);

    if (!m_vertex_attribute[vid].on_bbox_faces.empty()) return false;

    if (m_vertex_attribute[vid].m_is_rounded) return true;
    // try to round.
    // Note: no need to roll back.
    return r;
}

bool TetOptimizerMesh::smooth_after(const Tuple& t)
{
    optimization::SmoothVertexOptions opts;
    opts.w_amips = m_params.w_amips;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    // Off: the balanced warm-up's line search accepts tangential slides at any configured
    // w_amips (its objective contains no small weight), producing a weight-independent
    // surface-deviation floor (~4e-5 on triwild20k_202090, flat from w=1e-3 to 1e-12) that
    // vanishes with the warm-up off, at max energy unchanged to four digits and +1.8%
    // iterations on the 53 registered configs. Tangential mobility never needed the warm-up:
    // both the tangential gradient and Hessian scale with w_amips, so Newton cancels the
    // weight and vertices slide identically at any fitting strength.
    opts.two_stage = false;
    opts.smoothing_mode = m_params.smoothing_mode == "exact"
                              ? optimization::SmoothVertexOptions::SmoothingMode::Exact
                              : optimization::SmoothVertexOptions::SmoothingMode::Projected;
    opts.project_line_search_steps = m_params.project_line_search_steps;
    opts.project_line_search_nested_steps = m_params.project_line_search_nested_steps;

    return optimization::smooth_vertex_3d(*this, t, opts, m_solver.local(), &m_smooth_rejects);
}

void TetOptimizerMesh::smooth_all_vertices(const size_t n_iters)
{
    for (size_t i = 0; i < n_iters; ++i) {
        logger().info("==smoothing {}==", i);
        // Preserve TetWild's deterministic serial random-seed progression. The current
        // collector does not consume rand(), but keeping the state transition makes the move
        // behavior-neutral if its ordering is randomized again.
        static int rnd_seed = 0;
        srand(rnd_seed++);

        igl::Timer timer;
        timer.start();
        m_smooth_rejects.reset();
        std::vector<std::pair<std::string, Tuple>> collect_all_ops;
        if (m_params.skip_good_regions) {
            for (const size_t v : active_vertices()) {
                collect_all_ops.emplace_back("vertex_smooth", tuple_from_vertex(v));
            }
        } else {
            for (const Tuple& loc : get_vertices()) {
                collect_all_ops.emplace_back("vertex_smooth", loc);
            }
        }
        logger().info("vertex smoothing prepare time: {:.4}s", timer.getElapsedTime());
        logger().debug("#V = {}", collect_all_ops.size());
        run_pass(
            *this,
            PassLock::VertexRing,
            "vertex smoothing operation",
            [&](auto& executor, auto& mesh) { executor(mesh, collect_all_ops); });
        logger().info("\tsmooth: {}", m_smooth_rejects.to_string());

        if (m_params.debug_output) {
            write_optimization_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
        }
    }
}


std::shared_ptr<SampleEnvelope> TetOptimizerMesh::smoothing_containment_envelope(const size_t) const
{
    // Both applications keep one surface envelope. The pull energy may additionally select
    // an order-2 feature envelope, but containment remains against the surface envelope.
    return m_envelope;
}

std::vector<size_t> TetOptimizerMesh::all_vertex_ids() const
{
    const std::vector<Tuple> vs = get_vertices();
    std::vector<size_t> ids;
    ids.reserve(vs.size());
    for (const Tuple& v : vs) {
        ids.push_back(v.vid(*this));
    }
    return ids;
}

void TetOptimizerMesh::gradation_smooth_sizing(double grade, const std::vector<size_t>& seeds)
{
    utils::gradation_smooth_sizing(
        grade,
        seeds,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; },
        [this](size_t v) { return get_one_ring_vids_for_vertex_adj(v); });
}

/////////////////////////////////////////////////////////////////////
void TetOptimizerMesh::output_faces(
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
        matF.row(i) << (int)outface[i][0], (int)outface[i][1], (int)outface[i][2];
    }
    logger().info("Output face size {}", outface.size());
    igl::write_triangle_mesh(file, matV, matF);
}

std::tuple<double, double> TetOptimizerMesh::get_max_avg_energy()
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

        auto q = cell_quality(tup.tid(*this));
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

std::vector<size_t> TetOptimizerMesh::active_vertices() const
{
    return utils::active_vertices(
        vert_capacity(),
        tet_capacity(),
        [this](size_t tid) { return tuple_from_tet(tid).is_valid(*this); },
        [this](size_t tid) { return cell_quality(tid); },
        [this](size_t tid) { return oriented_tet_vids(tid); },
        active_quality_threshold(),
        [this](size_t vid) { return m_vertex_attribute[vid].m_is_on_surface; });
}

bool TetOptimizerMesh::is_inverted_f(const Tuple& loc) const
{
    auto vs = oriented_tet_vertices(loc);

    wmtk::utils::predicates::exactinit();
    auto res = wmtk::utils::predicates::orient3d(
        m_vertex_attribute[vs[0].vid(*this)].m_posf,
        m_vertex_attribute[vs[1].vid(*this)].m_posf,
        m_vertex_attribute[vs[2].vid(*this)].m_posf,
        m_vertex_attribute[vs[3].vid(*this)].m_posf);
    int result;
    if (res == wmtk::utils::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == wmtk::utils::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
        return false;
    return true;
}

bool TetOptimizerMesh::is_inverted(const std::array<size_t, 4>& vs) const
{
    // Return a positive value if the point pd lies below the
    // plane passing through pa, pb, and pc; "below" is defined so
    // that pa, pb, and pc appear in counterclockwise order when
    // viewed from above the plane.

    if (m_vertex_attribute[vs[0]].m_is_rounded && m_vertex_attribute[vs[1]].m_is_rounded &&
        m_vertex_attribute[vs[2]].m_is_rounded && m_vertex_attribute[vs[3]].m_is_rounded) {
        wmtk::utils::predicates::exactinit();
        auto res = wmtk::utils::predicates::orient3d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf,
            m_vertex_attribute[vs[3]].m_posf);
        int result;
        if (res == wmtk::utils::predicates::Orientation::POSITIVE)
            result = 1;
        else if (res == wmtk::utils::predicates::Orientation::NEGATIVE)
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

bool TetOptimizerMesh::is_inverted(const Tuple& loc) const
{
    auto vs = oriented_tet_vids(loc);
    return is_inverted(vs);
}

bool TetOptimizerMesh::round(const Tuple& v)
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

double TetOptimizerMesh::get_quality(const std::array<size_t, 4>& its) const
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

double TetOptimizerMesh::get_quality(const Tuple& loc) const
{
    auto its = oriented_tet_vids(loc);
    return get_quality(its);
}

bool TetOptimizerMesh::invariants(const std::vector<Tuple>& tets)
{
    return true;
}

std::vector<std::array<size_t, 3>> TetOptimizerMesh::get_faces_by_condition(
    std::function<bool(const FaceAttributes&)> cond) const
{
    auto res = std::vector<std::array<size_t, 3>>();
    for (auto f : get_faces()) {
        auto fid = f.fid(*this);
        if (cond(m_face_attribute[fid])) {
            auto tid = fid / 4, lid = fid % 4;
            auto verts = get_face_vertices(f);
            res.emplace_back( //
                std::array<size_t, 3>{
                    {verts[0].vid(*this), verts[1].vid(*this), verts[2].vid(*this)}});
        }
    }

    return res;
}

bool TetOptimizerMesh::is_edge_on_surface(const Tuple& loc)
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

int TetOptimizerMesh::edge_incident_surface_face_count(const Tuple& e)
{
    const size_t v1_id = e.vid(*this);
    const size_t v2_id = e.switch_vertex(*this).vid(*this);

    const auto tets = get_incident_tets_for_edge(e);
    std::vector<size_t> n_vids;
    for (const auto& t : tets) {
        const auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; ++j) {
            const size_t v = vs[j].vid(*this);
            if (v != v1_id && v != v2_id) n_vids.push_back(v);
        }
    }
    wmtk::vector_unique(n_vids);

    int count = 0;
    for (const size_t vid : n_vids) {
        auto [ftup, fid] = tuple_from_face({{v1_id, v2_id, vid}});
        (void)ftup;
        if (fid != static_cast<size_t>(-1) && m_face_attribute[fid].m_is_surface_fs) ++count;
    }
    return count;
}

bool TetOptimizerMesh::is_edge_on_bbox(const Tuple& loc)
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

bool TetOptimizerMesh::vertex_is_on_surface(const size_t vid) const
{
    return m_vertex_attribute.at(vid).m_is_on_surface;
}

bool TetOptimizerMesh::face_is_on_surface(const size_t fid) const
{
    return m_face_attribute.at(fid).m_is_surface_fs;
}

size_t TetOptimizerMesh::get_order_of_vertex(const size_t vid) const
{
    return m_vertex_attribute.at(vid).m_order;
}

double TetOptimizerMesh::get_length2(const Tuple& l) const
{
    auto& m = *this;
    auto& v1 = l;
    auto v2 = l.switch_vertex(m);
    double length =
        (m.m_vertex_attribute[v1.vid(m)].m_posf - m.m_vertex_attribute[v2.vid(m)].m_posf)
            .squaredNorm();
    return length;
}

} // namespace wmtk
