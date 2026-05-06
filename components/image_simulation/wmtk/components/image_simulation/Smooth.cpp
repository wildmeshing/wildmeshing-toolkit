
#include "ImageSimulationMesh.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <igl/edges.h>
#include <wmtk/utils/AMIPS.h>
#include <array>
#include <ipc/distance/point_triangle.hpp>
#include <unordered_set>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/BarrierEnergy.hpp>
#include <wmtk/optimization/DirichletEnergy.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/EnvelopeEnergy.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>


#include <limits>
#include <optional>

namespace wmtk::components::image_simulation {

bool ImageSimulationMesh::smooth_before(const Tuple& t)
{
    const bool r = round(t);

    const size_t vid = t.vid(*this);

    if (!m_vertex_attribute[vid].on_bbox_faces.empty()) return false;

    if (m_vertex_attribute[vid].m_is_rounded) return true;
    // try to round.
    // Note: no need to roll back.
    return r;
}

bool ImageSimulationMesh::smooth_after(const Tuple& t)
{
    // Newton iterations are encapsulated here.
    logger().trace("Newton iteration for vertex smoothing.");
    auto vid = t.vid(*this);

    const auto& VA = m_vertex_attribute;
    const auto& TA = m_tet_attribute;

    const auto locs = get_one_ring_tets_for_vertex(t);
    assert(!locs.empty());
    double max_quality = 0.;
    for (auto& tet : locs) {
        max_quality = std::max(max_quality, TA[tet.tid(*this)].m_quality);
        if (is_inverted_f(tet)) {
            // Neighbors that are not rounded could cause a tet to be inverted in floats
            // std::cout << "Inverted tet " << m_vertex_attribute[vid].m_posf.transpose() <<
            // std::endl;
            return false;
        }
    }

    auto& solver = m_solver.local();
    if (!solver) {
        solver = optimization::create_basic_solver();
    }

    const Vector3d old_pos = VA[vid].m_posf;

    auto amips_energy = get_amips_energy(t);

    std::shared_ptr<polysolve::nonlinear::Problem> total_energy = amips_energy;

    auto solve = [&]() {
        VectorXd x = VA[vid].m_posf;
        try {
            solver->minimize(*total_energy, x);
        } catch (const std::exception&) {
            // polysolve might throw errors that we want to ignore (e.g., line search failed)
        }
        m_vertex_attribute[vid].m_posf = x;
    };

    if (VA[vid].m_is_on_surface) {
        auto energy_sum = std::make_shared<optimization::EnergySum>();

        auto envelope_energy = get_envelope_energy(t);
        // do one solve without weights for amips and envelope
        if (m_params.w_amips > 0) {
            energy_sum->add_energy(amips_energy, 1. / m_params.w_amips);
        }
        if (m_params.w_envelope > 0) {
            energy_sum->add_energy(envelope_energy, 1. / m_params.w_envelope);
        }
        total_energy = energy_sum;
        solve();

        // second solve (with proper weights)
        energy_sum = std::make_shared<optimization::EnergySum>();

        if (m_params.w_smooth > 0) {
            auto smooth_energy = get_smooth_energy(t);
            if (smooth_energy) {
                energy_sum->add_energy(smooth_energy);
            }
        }
        if (m_params.w_amips > 0) {
            energy_sum->add_energy(amips_energy);
        }
        if (m_params.w_envelope > 0) {
            energy_sum->add_energy(envelope_energy);
        }
        // if (m_params.w_separate > 0) {
        //     auto barrier_energy = get_barrier_energy(t);
        //     if (barrier_energy) {
        //         double val = barrier_energy->value(old_pos);
        //         // only consider energy if it is non-zero at rest
        //         if (val > 0) {
        //             energy_sum->add_energy(barrier_energy);
        //         } else {
        //             logger().trace("Ignore barrier energy for zero rest state.");
        //         }
        //     }
        // }
        total_energy = energy_sum;
        solve();
    } else {
        // not on the surface
        solve();
    }

    logger().trace("old pos {} -> new pos {}", old_pos, VA[vid].m_posf);

    // check surface containment
    if (VA[vid].m_is_on_surface && !m_params.smooth_without_envelope) {
        // write_vtu_with_energies(fmt::format("debug_smooth_{}", debug_print_counter++));
        const simplex::SimplexCollection surf_assembles = get_surface_faces_for_vertex(vid);
        for (size_t i = 0; i < surf_assembles.faces().size(); ++i) {
            const simplex::Face& f = surf_assembles.faces()[i];

            std::array<Eigen::Vector3d, 3> face;
            face[0] = VA[f.vertices()[0]].m_posf;
            face[1] = VA[f.vertices()[1]].m_posf;
            face[2] = VA[f.vertices()[2]].m_posf;

            if (m_envelope->is_outside(face)) {
                return false;
            }
        }
    }

    // rational position must be updated before the inversion check!
    m_vertex_attribute[vid].m_pos = to_rational(VA[vid].m_posf);

    // quality
    auto max_after_quality = 0.;
    for (const Tuple& loc : locs) {
        if (is_inverted(loc)) {
            return false;
        }
        auto t_id = loc.tid(*this);
        m_tet_attribute[t_id].m_quality = get_quality(loc);
        max_after_quality = std::max(max_after_quality, TA[t_id].m_quality);
    }
    if (!VA[vid].m_is_on_surface) {
        if (max_after_quality > max_quality) {
            return false;
        }
    }

    return true;
}

void ImageSimulationMesh::smooth_all_vertices(const size_t n_iters = 1)
{
    build_mass_matrix();

    for (size_t i = 0; i < n_iters; ++i) {
        igl::Timer timer;
        double time;
        timer.start();
        auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
        for (auto& loc : get_vertices()) {
            collect_all_ops.emplace_back("vertex_smooth", loc);
        }
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing prepare time: {:.4}s", time);
        wmtk::logger().debug("Num verts {}", collect_all_ops.size());
        if (NUM_THREADS > 0) {
            timer.start();
            auto executor =
                wmtk::ExecutePass<ImageSimulationMesh>(wmtk::ExecutionPolicy::kPartition);
            executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
                return m.try_set_vertex_mutex_one_ring(e, task_id);
            };
            executor.num_threads = NUM_THREADS;
            executor(*this, collect_all_ops);
            time = timer.getElapsedTime();
            wmtk::logger().info("vertex smoothing operation time parallel: {:.4}s", time);
        } else {
            timer.start();
            auto executor = wmtk::ExecutePass<ImageSimulationMesh>(wmtk::ExecutionPolicy::kSeq);
            executor(*this, collect_all_ops);
            time = timer.getElapsedTime();
            wmtk::logger().info("vertex smoothing operation time serial: {:.4}s", time);
        }
        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }
    }

    // re-build envelope
    if (m_params.smooth_without_envelope) {
        logger().warn("Update envelope");
        MatrixXd V;
        V.resize(vert_capacity(), 3);
        V.setZero();
        for (size_t i = 0; i < vert_capacity(); ++i) {
            const Tuple v = tuple_from_vertex(i);
            if (!v.is_valid(*this)) {
                continue;
            }
            const size_t vid = v.vid(*this);
            V.row(i) = m_vertex_attribute.at(vid).m_posf;
        }

        const auto surf_faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });
        MatrixXi F;
        F.resize(surf_faces.size(), 3);
        for (size_t i = 0; i < surf_faces.size(); ++i) {
            F.row(i) =
                Vector3i((int)surf_faces[i][0], (int)surf_faces[i][1], (int)surf_faces[i][2]);
        }

        m_envelope = nullptr;
        m_V_envelope.clear();
        m_F_envelope.clear();
        init_envelope(V, F);
    }
}

void ImageSimulationMesh::build_mass_matrix()
{
    MatrixXd V;
    MatrixXi F;

    V.resize(vert_capacity(), 3);
    for (size_t i = 0; i < V.rows(); ++i) {
        V.row(i) = m_vertex_attribute.at(i).m_posf;
    }

    const auto faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });
    F.resize(faces.size(), 3);
    for (size_t i = 0; i < F.rows(); ++i) {
        F.row(i) = Vector3i(faces[i][0], faces[i][1], faces[i][2]);
    }

    if (F.rows() == 0) {
        logger().warn("No surface faces found. Mass matrix cannot be built.");
        return;
    }
    optimization::BiharmonicEnergy3D::global_mass_and_stiffness(
        V,
        F,
        m_surface_mass,
        m_surface_stiffness);
}

std::vector<std::array<double, 12>> ImageSimulationMesh::get_amips_assembles(const Tuple& t) const
{
    const size_t vid = t.vid(*this);
    const auto locs = get_one_ring_tets_for_vertex(t);

    std::vector<std::array<double, 12>> assembles(locs.size());
    int loc_id = 0;

    for (const Tuple& loc : locs) {
        auto& T = assembles[loc_id];
        auto t_id = loc.tid(*this);
        assert(!is_inverted_f(loc));

        std::array<size_t, 4> local_verts = oriented_tet_vids(t_id);
        local_verts = wmtk::orient_preserve_tet_reorder(local_verts, vid);

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                T[i * 3 + j] = m_vertex_attribute[local_verts[i]].m_posf[j];
            }
        }
        loc_id++;
    }

    return assembles;
}

std::shared_ptr<polysolve::nonlinear::Problem> ImageSimulationMesh::get_amips_energy(
    const Tuple& t) const
{
    const double w = m_params.w_amips > 0 ? m_s_amips * m_params.w_amips : 1;

    const auto assembles = get_amips_assembles(t);
    auto amips_energy = std::make_shared<optimization::AMIPSEnergy3D>(assembles, w);
    assert(amips_energy->initial_position() == m_vertex_attribute.at(t.vid(*this)).m_posf);
    return amips_energy;
}

std::shared_ptr<polysolve::nonlinear::Problem> ImageSimulationMesh::get_smooth_energy(
    const Tuple& t) const
{
    const auto& VA = m_vertex_attribute;
    const size_t vid = t.vid(*this);

    std::vector<size_t> adj;
    optimization::BiharmonicEnergy3D::adjacency_from_stiffness(vid, m_surface_stiffness, adj);

    if (adj.empty()) {
        return nullptr;
    }

    MatrixXd pts;
    pts.resize(adj.size() + 1, 3);
    pts.row(0) = VA[vid].m_posf;
    for (size_t j = 0; j < adj.size(); ++j) {
        pts.row(j + 1) = VA[adj[j]].m_posf;
    }

    double M;
    VectorXd L_w;
    optimization::BiharmonicEnergy3D::extract_local_mass_and_stiffness(
        vid,
        m_surface_mass,
        m_surface_stiffness,
        M,
        L_w);

    const double w = m_s_smooth * m_params.w_smooth;

    auto smooth_energy = std::make_shared<optimization::BiharmonicEnergy3D>(pts, M, L_w, w);

    return smooth_energy;
}

std::shared_ptr<polysolve::nonlinear::Problem> ImageSimulationMesh::get_envelope_energy(
    const Tuple& t) const
{
    size_t vid = t.vid(*this);
    // const auto& M = m_surface_mass.coeff(vid, vid);
    const double w = m_s_envelope * m_params.w_envelope;

    auto env = m_envelope_orig;
    if (m_vertex_attribute[vid].m_order > 1) {
        env = m_order_2_edge_envelope;
    }

    if (!env) {
        log_and_throw_error(
            "Envelope was not initialized. Vertex was of order {}",
            m_vertex_attribute[vid].m_order);
    }

    auto envelope_energy =
        std::make_shared<optimization::EnvelopeEnergy3D>(env, w, !m_params.smooth_without_envelope);
    return envelope_energy;
}

std::shared_ptr<polysolve::nonlinear::Problem> ImageSimulationMesh::get_barrier_energy(
    const Tuple& t,
    const bool use_full_surface) const
{
    const auto& VA = m_vertex_attribute;
    const size_t vid = t.vid(*this);

    if (!VA[vid].m_is_on_surface) {
        return nullptr;
    }

    // const auto& M = m_surface_mass.coeff(vid, vid);
    // if (M <= 0) {
    //     return nullptr;
    // }

    // barrier energy
    MatrixXd V_barrier;
    MatrixXi F_barrier;
    size_t vid_barrier;
    if (!use_full_surface) {
        substructure_region(t, V_barrier, F_barrier, vid_barrier);
    } else {
        // this is horribly expensive and is only here for testing

        std::vector<Vector3d> surf_points;
        std::vector<size_t> global_to_local_vid_map(vert_capacity());
        for (size_t i = 0; i < vert_capacity(); ++i) {
            const Tuple v = tuple_from_vertex(i);
            if (!v.is_valid(*this)) {
                continue;
            }
            const size_t _vid = v.vid(*this);
            if (!m_vertex_attribute.at(_vid).m_is_on_surface) {
                continue;
            }
            surf_points.push_back(m_vertex_attribute.at(_vid).m_posf);
            global_to_local_vid_map[_vid] = surf_points.size() - 1;
        }

        V_barrier.resize(surf_points.size(), 3);
        for (size_t i = 0; i < surf_points.size(); ++i) {
            V_barrier.row(i) = surf_points[i];
        }

        const auto faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });
        F_barrier.resize(faces.size(), 3);
        for (size_t i = 0; i < faces.size(); ++i) {
            const size_t v0 = global_to_local_vid_map[faces[i][0]];
            const size_t v1 = global_to_local_vid_map[faces[i][1]];
            const size_t v2 = global_to_local_vid_map[faces[i][2]];
            F_barrier.row(i) = Vector3i(v0, v1, v2);
        }

        vid_barrier = global_to_local_vid_map[vid];
    }

    if (F_barrier.rows() == 0) {
        return nullptr;
    }

    MatrixXi E_barrier;
    igl::edges(F_barrier, E_barrier);

    auto barrier_energy = std::make_shared<optimization::BarrierEnergy3D>(
        V_barrier,
        E_barrier,
        F_barrier,
        vid_barrier,
        m_params.dhat,
        m_s_barrier * m_params.w_separate);

    return barrier_energy;
}

void ImageSimulationMesh::substructure_region(
    const Tuple& v_tuple,
    MatrixXd& V,
    MatrixXi& F,
    size_t& vid_local) const
{
    const auto& VA = m_vertex_attribute;
    const size_t vid_global = v_tuple.vid(*this);

    if (!VA[vid_global].m_is_on_surface) {
        log_and_throw_error(
            "Cannot compute substructure region for vertex that is not on the surface");
        // We actually could compute this but it doesn't make sense. Remove this if-statement if we
        // should ever need to compute regions for vertices not on the surface.
    }

    const Vector3d& p0 = VA[vid_global].m_posf;

    double l_max = 0; // longest incident edge length
    for (const size_t& v1 : get_one_ring_vids_for_vertex(vid_global)) {
        const Vector3d& p1 = VA[v1].m_posf;
        const double l_squared = (p1 - p0).squaredNorm();
        l_max = std::max(l_max, l_squared);
    }
    l_max = std::sqrt(l_max);

    const double r = 1.5 * m_params.dhat + l_max; // region radius
    const double r2 = r * r;

    simplex::SimplexCollection candidates; // all faces within the region

    // make a BFS to find faces within distance
    std::unordered_set<size_t> visited;
    std::queue<size_t> q;
    q.push(vid_global);
    visited.insert(vid_global);

    while (!q.empty()) {
        const size_t v_a = q.front();
        q.pop();

        const Vector3d& p_a = VA[v_a].m_posf;
        const double d2_a = (p0 - p_a).squaredNorm();

        for (const size_t& tid : get_one_ring_tids_for_vertex(vid_global)) {
            // get all faces from tet
            simplex::Tet tet = simplex_from_tet(tid);
            const auto fs = tet.faces();

            for (size_t i = 0; i < 4; ++i) {
                const Vector3d& t1 = VA[fs[i].vertices()[0]].m_posf;
                const Vector3d& t2 = VA[fs[i].vertices()[1]].m_posf;
                const Vector3d& t3 = VA[fs[i].vertices()[2]].m_posf;

                // if p_a is within the region, skip distance test for triangles
                if (d2_a <= r2) {
                    const double d2 = ipc::point_triangle_distance(p0, t1, t2, t3);
                    if (d2 > r2) {
                        // triangle is too far away
                        continue;
                    }
                }

                candidates.add(fs[i]);
                // add vertices to queue
                for (size_t j = 0; j < 3; ++j) {
                    const size_t v_b = fs[i].vertices()[j];
                    if (visited.count(v_b) == 0) {
                        q.push(v_b);
                        visited.insert(v_b);
                    }
                }
            }
        }
    }
    candidates.sort_and_clean();

    std::vector<simplex::Face> faces; // faces on the surface
    for (const auto& f : candidates.faces()) {
        const auto [tup, fid] = tuple_from_face(f.vertices());
        if (face_is_on_surface(fid)) {
            faces.push_back(f);
        }
    }

    std::unordered_set<size_t> region_vertices;
    for (const auto& f : faces) {
        region_vertices.insert(f.vertices()[0]);
        region_vertices.insert(f.vertices()[1]);
        region_vertices.insert(f.vertices()[2]);
    }

    // build V and F
    std::vector<Vector3d> points;
    std::unordered_map<size_t, size_t> global_to_local_vid_map;
    for (const size_t i : region_vertices) {
        const Tuple v = tuple_from_vertex(i);
        assert(v.is_valid(*this));
        assert(VA[i].m_is_on_surface);
        points.push_back(VA[i].m_posf);
        global_to_local_vid_map[i] = points.size() - 1;
    }

    V.resize(points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        V.row(i) = points[i];
    }

    F.resize(faces.size(), 3);
    for (size_t i = 0; i < faces.size(); ++i) {
        const size_t v0 = global_to_local_vid_map[faces[i].vertices()[0]];
        const size_t v1 = global_to_local_vid_map[faces[i].vertices()[1]];
        const size_t v2 = global_to_local_vid_map[faces[i].vertices()[2]];
        F.row(i) = Vector3i(v0, v1, v2);
    }

    vid_local = global_to_local_vid_map[vid_global];
}

void ImageSimulationMesh::init_separation_weight()
{
    if (m_params.preserve_topology) {
        // no separation for not preserving topology
        m_params.w_separate = 0;
    }
    if (m_params.separation_factor <= 0) {
        m_params.w_separate = 0;
        logger().warn(
            "Separation factor is {} -> No separation. IPC barrier weight = {}",
            m_params.separation_factor,
            m_params.w_separate);
        return;
    }

    /**
     * This assumes that the barrier term is the clamped log barrier term:
     * E_B = b(d) = -(d-\hat{d})^2\ln\left(\frac{d}{\hat{d}}\right)
     *
     * E_B' = b'(\hat{d}/2) = \hat{d} * (\ln(2) - 0.5)
     *
     * barrier scaling: s_B = 1/l^4
     * envelope scaling: s_E = 1 / (l * eps^2)
     *
     * l is the bounding box diagonal.
     *
     * The envelope energy E_E = \Vert p - p_0 \Vert^2
     * E_E'(\hat{d}/2) = \hat{d}
     *
     * We compute w_E from
     * s_B w_B E_B' = s_E w_E E_E'
     *
     * w_B = w_E l^3 / (eps^2 * (\ln(2) - 0.5))
     */

    const double l3 = m_params.diag_l3;
    const double eps2 = m_params.eps * m_params.eps;
    const double ln2 = std::log(2.0);

    m_params.w_separate = m_params.w_envelope * l3 / (eps2 * (ln2 - 0.5));

    logger().info("IPC barrier weight = {}, dhat = {}", m_params.w_separate, m_params.dhat);

    if (m_params.w_separate < 0) {
        logger().error("Negative separation weight: {}. Setting weight to 0.", m_params.w_separate);
        m_params.w_separate = 0;
    }
}

} // namespace wmtk::components::image_simulation