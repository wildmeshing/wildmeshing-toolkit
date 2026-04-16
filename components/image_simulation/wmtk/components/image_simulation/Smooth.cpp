
#include "ImageSimulationMesh.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS.h>
#include <array>
#include <wmtk/optimization/AMIPSEnergy.hpp>
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

    const Vector3d old_pos = VA[vid].m_posf;

    // m_vertex_attribute[vid].m_posf = wmtk::newton_method_from_stack(
    //     assembles,
    //     wmtk::AMIPS_energy,
    //     wmtk::AMIPS_jacobian,
    //     wmtk::AMIPS_hessian);

    std::shared_ptr<polysolve::nonlinear::Problem> total_energy;

    auto amips_energy = get_amips_energy(t);

    total_energy = amips_energy;

    //// project to open boundary
    // if (VA[vid].m_is_on_open_boundary) {
    //     assert(m_open_boundary_envelope.initialized());
    //    // project to envelope
    //    Vector3d project;
    //    m_open_boundary_envelope.nearest_point(VA[vid].m_posf, project);
    //    m_vertex_attribute[vid].m_posf = project;
    //}

    auto solve = [&]() {
        VectorXd x = VA[vid].m_posf;
        try {
            m_solver->minimize(*total_energy, x);
        } catch (const std::exception&) {
            // polysolve might throw errors that we want to ignore (e.g., line search failed)
        }
        m_vertex_attribute[vid].m_posf = x;
    };

    if (VA[vid].m_is_on_surface) {
        // if (VA[vid].m_order == 3) {
        //     logger().warn("Ignoring vertices with order {}", VA[vid].m_order);
        //     return false;
        // }

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
    assert(m_solver);
    build_mass_matrix();

    //// the order is randomized in every iteration but deterministic when executed sequentially
    // static int rnd_seed = 0;
    // srand(rnd_seed++);

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
            // executor.priority = [&](auto& m, auto op, auto& t) -> double { return rand(); };
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

void ImageSimulationMesh::smooth_input(const int n_iterations)
{
    const auto vertices = get_vertices();

    for (int i = 0; i < n_iterations; ++i) {
        for (const Tuple& t : vertices) {
            const size_t vid = t.vid(*this);
            if (!m_vertex_attribute[vid].on_bbox_faces.empty()) {
                continue;
            }
            if (!m_vertex_attribute[vid].m_is_rounded) {
                continue;
            }

            const auto locs = get_one_ring_tets_for_vertex(t);

            if (m_vertex_attribute[vid].m_is_on_surface) {
                // on surface --> try laplacian smoothing

                std::vector<std::array<Vector3d, 3>>
                    neighbor_assemble; // incident surface triangles
                std::set<size_t> unique_fid;
                for (const Tuple& t : locs) {
                    for (int j = 0; j < 4; j++) {
                        const Tuple f_t = tuple_from_face(t.tid(*this), j);
                        const size_t fid = f_t.fid(*this);
                        if (!m_face_attribute[fid].m_is_surface_fs) {
                            continue;
                        }
                        const auto [it, suc] = unique_fid.emplace(fid);
                        if (!suc) {
                            continue;
                        }
                        const auto vs = get_face_vertices(f_t);
                        auto vs_id = std::array<size_t, 3>();
                        for (int k = 0; k < 3; k++) {
                            vs_id[k] = vs[k].vid(*this);
                        }
                        for (int k : {1, 2}) {
                            if (vs_id[k] == vid) {
                                std::swap(vs_id[k], vs_id[0]);
                            }
                        }
                        if (vs_id[0] != vid) {
                            continue; // does not contain point of interest
                        }
                        std::array<Vector3d, 3> coords;
                        for (int k = 0; k < 3; k++) {
                            coords[k] = m_vertex_attribute[vs_id[k]].m_posf;
                        }
                        neighbor_assemble.emplace_back(coords);
                    }
                }

                // TODO smooth
                Vector3d p_lap = Vector3d::Zero();
                for (const auto& n : neighbor_assemble) {
                    p_lap += n[1] + n[2];
                }
                p_lap /= (neighbor_assemble.size() * 2);

                const Vector3d p_old = m_vertex_attribute[vid].m_posf;

                bool success = true;
                const int n_bisections = 10;
                for (int j = 0; j < n_bisections; ++j) {
                    // try to find a valid position
                    double u = 1.0 / (1 << j);
                    Vector3d p = u * p_lap + (1.0 - u) * p_old;

                    // update position of vid in neighbor_assamble
                    for (auto& n : neighbor_assemble) {
                        n[0] = p;
                    }
                    success = true;
                    for (const auto& n : neighbor_assemble) {
                        if (m_envelope->is_outside(n)) {
                            success = false;
                            break;
                        }
                    }

                    if (!success) {
                        continue;
                    }

                    // check for tet validity
                    m_vertex_attribute[vid].m_posf = p;
                    for (const Tuple& loc : locs) {
                        if (is_inverted(loc)) {
                            success = false;
                            break;
                        }
                    }

                    if (success) {
                        m_vertex_attribute[vid].m_pos = to_rational(p);
                        break;
                    } else {
                        m_vertex_attribute[vid].m_posf = p_old;
                    }
                }

                for (const Tuple& loc : locs) {
                    if (is_inverted(loc)) {
                        log_and_throw_error("Input surface laplacian smooth caused inversion");
                    }
                }

            } else {
                // interior --> amips optimization

                auto max_quality = 0.;
                for (const Tuple& tet : locs) {
                    max_quality = std::max(max_quality, m_tet_attribute[tet.tid(*this)].m_quality);
                }

                assert(locs.size() > 0);
                std::vector<std::array<double, 12>> assembles(locs.size());
                auto loc_id = 0;

                for (auto& loc : locs) {
                    auto& T = assembles[loc_id];
                    auto t_id = loc.tid(*this);

                    assert(!is_inverted(loc));
                    auto local_tuples = oriented_tet_vertices(loc);
                    std::array<size_t, 4> local_verts;
                    for (auto i = 0; i < 4; i++) {
                        local_verts[i] = local_tuples[i].vid(*this);
                    }

                    local_verts = wmtk::orient_preserve_tet_reorder(local_verts, vid);

                    for (auto i = 0; i < 4; i++) {
                        for (auto j = 0; j < 3; j++) {
                            T[i * 3 + j] = m_vertex_attribute[local_verts[i]].m_posf[j];
                        }
                    }
                    loc_id++;
                }

                auto old_pos = m_vertex_attribute[vid].m_posf;

                m_vertex_attribute[vid].m_posf = wmtk::newton_method_from_stack(
                    assembles,
                    wmtk::AMIPS_energy,
                    wmtk::AMIPS_jacobian,
                    wmtk::AMIPS_hessian);

                // quality
                double max_after_quality = 0.;
                for (const Tuple& loc : locs) {
                    if (is_inverted(loc)) {
                        log_and_throw_error("Input smoothing caused inversion");
                    }
                    const size_t t_id = loc.tid(*this);
                    m_tet_attribute[t_id].m_quality = get_quality(loc);
                    max_after_quality =
                        std::max(max_after_quality, m_tet_attribute[t_id].m_quality);
                }
                // if (max_after_quality > max_quality) {
                //     log_and_throw_error("Input smoothing reduced tet quality");
                // }

                m_vertex_attribute[vid].m_pos = to_rational(m_vertex_attribute[vid].m_posf);
            }
        }
        //
    }
    //
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
    const auto& M = m_surface_mass.coeff(vid, vid);
    const double w = m_s_envelope * m_params.w_envelope;

    auto env = m_envelope_orig;
    if (m_vertex_attribute[vid].m_order > 1) {
        env = m_order_2_edge_envelope;
    }

    auto envelope_energy =
        std::make_shared<optimization::EnvelopeEnergy3D>(env, w, !m_params.smooth_without_envelope);
    return envelope_energy;
}

} // namespace wmtk::components::image_simulation