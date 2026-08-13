
#include "TetWildMesh.h"

#include <tuple>

#include "wmtk/utils/Rational.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/WindingNumber.hpp>
#include <wmtk/utils/io.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <spdlog/fmt/ostr.h>
#include <spdlog/fmt/bundled/format.h>
#include <wmtk/utils/predicates.hpp>
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
#include <queue>
#include <unordered_set>

#include "orig/Args.h"
#include "orig/MeshRefinement.h"
#include "orig/State.h"

namespace wmtk::components::tetwild {

void TetWildMesh::optimization_sanity_checks_extra()
{
    // Boundary edges may temporarily leave the envelope during collapse to remove thin
    // ribbons. Preserve TetWild's diagnostic for every other open surface edge.
    std::vector<int> edge_on_open_boundary(6 * tet_capacity(), 0);
    for (const Tuple& f : get_faces()) {
        if (!m_face_attribute[f.fid(*this)].m_is_surface_fs) continue;
        ++edge_on_open_boundary[f.eid(*this)];
        ++edge_on_open_boundary[f.switch_edge(*this).eid(*this)];
        ++edge_on_open_boundary[f.switch_vertex(*this).switch_edge(*this).eid(*this)];
    }

    for (const Tuple& e : get_edges()) {
        if (edge_on_open_boundary[e.eid(*this)] != 1 || is_open_boundary_edge(e)) continue;
        const size_t v1 = e.vid(*this);
        const size_t v2 = e.switch_vertex(*this).vid(*this);
        if (!m_vertex_extra[v1].m_is_on_open_boundary ||
            !m_vertex_extra[v2].m_is_on_open_boundary) {
            continue;
        }
        logger().warn("Boundary edge ({},{}) is outside the envelope.", v1, v2);
    }
}

std::shared_ptr<SampleEnvelope> TetWildMesh::smoothing_energy_envelope(const size_t vid) const
{
    // Order 2 means the vertex is on a surface boundary or a non-manifold edge. This is
    // broader than the old m_is_on_open_boundary flag, which covered only open boundaries.
    if (get_order_of_vertex(vid) >= 2 && m_order2_envelope && m_order2_envelope->initialized()) {
        return m_order2_envelope;
    }
    return m_envelope;
}


void TetWildMesh::mesh_improvement_legacy(int max_its)
{
    logger().set_level(spdlog::level::level_enum::debug);

    // m_envelope is a SampleEnvelope already; the dynamic_cast these lines used to do was
    // a no-op left over from when it was held as the Envelope base.
    SampleEnvelope* env = m_envelope.get();
    SampleEnvelope* env_b = m_envelope.get();
    if (env == nullptr) {
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
            if (m_vertex_extra[vid].m_is_on_open_boundary) {
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
                v[j] = (int)vi[j];
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
            const auto& VX = m_vertex_extra[i];
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
            v.is_on_boundary = VX.m_is_on_open_boundary;
            v.is_on_surface = VA.m_is_on_surface;
            v.is_rounded = VA.m_is_rounded;
            for (const size_t tid : get_one_ring_tids_for_vertex(i)) {
                v.conn_tets.insert((int)tid);
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
            auto& VX = m_vertex_extra[i];
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
            VX.m_is_on_open_boundary = v.is_on_boundary;
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

size_t TetWildMesh::refine_sizing_around_worst(double max_energy)
{
    const int n_rings = std::max(0, m_params.stuck_refine_rings);
    // Clamped above, as the original TetWild's updateScalarField target does by ratcheting
    // down from 1e6. Without the clamp a single
    // degenerate tet (quality MAX_ENERGY, reported as 4.6e16) sets filter_energy to 4.6e14,
    // and select_worst_cells then picks out only the degenerate tets -- refinement stops
    // seeing the merely-bad ones it exists to fix.
    const double filter_energy = std::min(std::max(max_energy / 100, m_params.stop_energy), 100.);

    // m_quality stores AMIPS^3, so the energy the "max energy" refers to is its cube root.
    const auto worst = utils::select_worst_cells(
        tet_capacity(),
        [this](size_t tid) { return tuple_from_tet(tid).is_valid(*this); },
        [this](size_t tid) { return std::cbrt(m_tet_attribute[tid].m_quality); },
        filter_energy,
        m_params.stuck_refine_num_worst);

    if (worst.empty()) {
        return 0;
    }

    // If force-split is on, record the LONGEST edge of each worst tet. split_all_edges
    // force-splits exactly those edges (bypasses the length gate), so a stuck sliver's
    // long edge is split immediately -- WITHOUT changing the sizing field.
    m_force_split_edges.clear();
    if (m_params.stuck_refine_force_split) {
        for (const auto& [_, tid] : worst) {
            const auto e = utils::longest_edge(
                oriented_tet_vids(tid),
                [this](size_t vid) -> const Vector3d& { return m_vertex_attribute[vid].m_posf; });
            m_force_split_edges.insert(e);
        }
    }

    // Seed the region with the worst tets' vertices, then BFS n_rings hops.
    std::vector<size_t> seeds;
    seeds.reserve(4 * worst.size());
    for (const auto& [_, tid] : worst) {
        for (const size_t v : oriented_tet_vids(tid)) {
            seeds.push_back(v);
        }
    }
    const auto one_ring = [this](size_t v) { return get_one_ring_vids_for_vertex_adj(v); };
    const auto region = utils::grow_vertex_region(seeds, n_rings, one_ring);

    const auto sizing = [this](size_t v) -> double& {
        return m_vertex_attribute[v].m_sizing_scalar;
    };

    // Apply the multiplicative refinement, clamped at the floor.
    const auto refined = utils::apply_sizing_refinement(
        region,
        m_params.stuck_refine_factor,
        m_params.stuck_refine_min_scalar,
        sizing);

    // Grade the refined region into its surroundings (monotone, only lowers).
    gradation_smooth_sizing(m_params.stuck_refine_gradation, refined);

    // m_quality stores AMIPS^3; report its cube root to match the "max energy".
    logger().info(
        "[stuck-refine] worst {} tets (maxE {:.4}), refined {} of {} region vertices, "
        "filter_energy {:.4}",
        worst.size(),
        worst.back().first,
        refined.size(),
        region.size(),
        filter_energy);
    return refined.size();
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

Eigen::MatrixXd TetWildMesh::tet_barycenters(const std::vector<Tuple>& tets) const
{
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(tets.size(), 3);
    for (size_t i = 0; i < tets.size(); i++) {
        const auto vs = oriented_tet_vids(tets[i]);
        for (size_t v : vs) C.row(i) += m_vertex_attribute[v].m_posf;
        C.row(i) /= 4;
    }
    return C;
}

void TetWildMesh::compute_winding_number(
    const std::vector<Tuple>& tets,
    const Eigen::MatrixXd& barycenters,
    const std::vector<Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    if (!vertices.empty()) {
        V.resize(vertices.size(), 3);
        F.resize(faces.size(), 3);

        for (size_t i = 0; i < (size_t)V.rows(); i++) {
            V.row(i) = vertices[i];
        }
        for (size_t i = 0; i < (size_t)F.rows(); i++) {
            for (size_t j = 0; j < 3; j++) {
                F(i, j) = (int)faces[i][j];
            }
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
            F.row(i) << (int)outface[i][0], (int)outface[i][1], (int)outface[i][2];
        }
        // logger().info("Output face size {}", outface.size());
        auto F0 = F;
        Eigen::VectorXi C;
        bfs_orient(F0, F, C);
        // logger().info("BFS orient {}", F.rows());
    }

    // `tets` and their `barycenters` are precomputed once by the caller and shared
    // across the winding-number passes (input / tracked / per-input).
    Eigen::VectorXd W;
    wmtk::utils::winding_number(V, F, barycenters, W, NUM_THREADS);

    if (W.maxCoeff() <= 0.5) {
        // all removed, let's invert.
        logger().info("Correcting winding number");
        for (auto i = 0; i < F.rows(); i++) {
            auto temp = F(i, 0);
            F(i, 0) = F(i, 1);
            F(i, 1) = temp;
        }
        wmtk::utils::winding_number(V, F, barycenters, W, NUM_THREADS);
    }

    if (W.maxCoeff() <= 0.5) {
        logger().critical("Still Inverting..., Empty Output");
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

void TetWildMesh::compute_winding_numbers(
    const std::vector<std::string>& input_paths,
    const std::vector<Tuple>& tets,
    const Eigen::MatrixXd& barycenters,
    const std::vector<Vector3d>& in_vertices,
    const std::vector<std::array<size_t, 3>>& in_faces)
{
    // Single-input fast path: with one input surface, the per-input winding number is
    // evaluated from exactly the same surface (in_vertices/in_faces) and the same query
    // barycenters as the input-surface winding number that compute_winding_number(...)
    // has already computed and stored in m_winding_number_input. Recomputing it here
    // repeats the full winding-number evaluation (the dominant cost of the finalize
    // phase on large meshes) for an identical result, so reuse the stored value.
    //
    // Precondition: compute_winding_number(tets, barycenters, in_vertices, in_faces) has
    // run before this call (as tetwild does), so m_winding_number_input is populated.
    // Reusing it also sidesteps a stale/racy value that igl's WindingNumberAABB static
    // cache can return on a second, independent evaluation of the same surface.
    if (input_paths.size() == 1 && !in_vertices.empty() && !in_faces.empty()) {
        for (int i = 0; i < (int)tets.size(); ++i) {
            const size_t tid = tets[i].tid(*this);
            m_tet_attribute[tid].m_winding_number_per_input.assign(
                1,
                m_tet_attribute[tid].m_winding_number_input);
        }
        return;
    }

    // Multiple inputs: evaluate each input surface's winding number independently,
    // reading every file from disk (the in-memory surface is only the merged input).
    for (size_t p = 0; p < input_paths.size(); ++p) {
        MatrixXd V;
        MatrixXi F;
        {
            MatrixXd inV;
            MatrixXi inF;
            igl::read_triangle_mesh(input_paths[p], inV, inF);
            VectorXi _I;
            igl::remove_unreferenced(inV, inF, V, F, _I);
        }
        assert(V.cols() == 3);
        assert(F.cols() == 3);

        // compute winding number for V,F using the shared barycenters
        Eigen::VectorXd W;
        wmtk::utils::winding_number(V, F, barycenters, W, NUM_THREADS);

        if (W.maxCoeff() <= 0.5) {
            // all removed, let's invert.
            logger().info("Correcting winding number");
            for (auto i = 0; i < F.rows(); i++) {
                auto temp = F(i, 0);
                F(i, 0) = F(i, 1);
                F(i, 1) = temp;
            }
            wmtk::utils::winding_number(V, F, barycenters, W, NUM_THREADS);
        }

        if (W.maxCoeff() <= 0.5) {
            logger().warn("No winding number above 0.5 for input_path {}", input_paths[p]);
        }

        // store winding number in mesh
        for (int i = 0; i < (int)tets.size(); ++i) {
            const size_t tid = tets[i].tid(*this);
            m_tet_attribute[tid].m_winding_number_per_input.push_back(W(i));
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

    // per input winding number
    if (!tets.empty()) {
        const size_t n = m_tet_attribute[tets[0].tid(*this)].m_winding_number_per_input.size();
        for (size_t j = 0; j < n; ++j) {
            msh.add_tet_attribute<1>(fmt::format("wn_{}", j), [&](size_t i) {
                return std::cbrt(m_tet_attribute[i].m_winding_number_per_input[j]);
            });
        }
    }

    msh.save(file, true);
}


bool TetWildMesh::is_vertex_on_boundary(const size_t e0)
{
    if (!m_vertex_extra.at(e0).m_is_on_open_boundary) {
        return false;
    }

    const auto neigh_vids = get_one_ring_vids_for_vertex(e0);
    const auto e0_tids = get_one_ring_tids_for_vertex(e0);

    for (const size_t e1 : neigh_vids) {
        if (!m_vertex_extra.at(e1).m_is_on_open_boundary) {
            continue;
        }
        int cnt = 0;
        for (size_t t_id : e0_tids) {
            const auto vs = oriented_tet_vids(t_id);
            std::array<int, 4> opp_js; // DZ: all vertices that are adjacent to e1 except for e2
            int ii = 0;
            for (int j = 0; j < 4; j++) {
                if (vs[j] == e0 || vs[j] == e1) {
                    continue;
                }
                opp_js[ii++] = j;
            }
            // DZ: if the tet contains e1 and e2, then ii == 2
            if (ii != 2) {
                continue;
            }
            // DZ: opp_js vertices form a tet together with v1,v2
            if (m_vertex_attribute.at(vs[opp_js[0]]).m_is_on_surface) {
                const auto [f0_tup, f0_id] = tuple_from_face({{e0, e1, vs[opp_js[0]]}});
                if (m_face_attribute.at(f0_id).m_is_surface_fs) {
                    cnt++;
                }
            }
            if (m_vertex_attribute.at(vs[opp_js[1]]).m_is_on_surface) {
                const auto [f1_tup, f1_id] = tuple_from_face({{e0, e1, vs[opp_js[1]]}});
                if (m_face_attribute.at(f1_id).m_is_surface_fs) {
                    cnt++;
                }
            }
            if (cnt > 2) {
                break;
            }
        }
        // all faces are visited twice, so cnt == 2 means there is one boundary face
        if (cnt == 2) {
            // this is a boundary edge
            return true;
        }
    }

    return false;
}


void TetWildMesh::init_vertex_order()
{
    // Per-vertex, independent: compute_vertex_order is const (reads connectivity
    // only) and each vid writes only its own m_order slot.
    const std::vector<Tuple> vs = get_vertices();
    threading::parallel_for(
        threading::range(0, vs.size()),
        [&](const threading::range& range) {
            for (size_t k = range.begin(); k < range.end(); ++k) {
                const size_t vid = vs[k].vid(*this);
                m_vertex_attribute[vid].m_order = compute_vertex_order(vid);
            }
        },
        NUM_THREADS);
}

int TetWildMesh::flood_fill()
{
    int current_id = 0;
    auto tets = get_tets();
    std::vector<char> visited(tet_capacity(), 0);

    for (const Tuple& t : tets) {
        size_t tid = t.tid(*this);
        if (visited[tid]) continue;

        // std::cout << "for loop current id: " << current_id << std::endl;

        visited[tid] = 1;

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
                if (!visited[(*oppo_t).tid(*this)]) bfs_queue.push(*oppo_t);
            }
        }
        if (!m_face_attribute[f2.fid(*this)].m_is_surface_fs) {
            // std::cout << "in 2" << std::endl;
            auto oppo_t = f2.switch_tetrahedron(*this);
            if (oppo_t.has_value()) {
                if (!visited[(*oppo_t).tid(*this)]) bfs_queue.push(*oppo_t);
            }
        }
        if (!m_face_attribute[f3.fid(*this)].m_is_surface_fs) {
            // std::cout << "in 3" << std::endl;
            auto oppo_t = f3.switch_tetrahedron(*this);
            if (oppo_t.has_value()) {
                if (!visited[(*oppo_t).tid(*this)]) bfs_queue.push(*oppo_t);
            }
        }
        if (!m_face_attribute[f4.fid(*this)].m_is_surface_fs) {
            // std::cout << "in 4" << std::endl;
            auto oppo_t = f4.switch_tetrahedron(*this);
            if (oppo_t.has_value()) {
                if (!visited[(*oppo_t).tid(*this)]) bfs_queue.push(*oppo_t);
            }
        }

        // std::cout << "while loop current id: ";

        while (!bfs_queue.empty()) {
            auto tmp = bfs_queue.front();
            bfs_queue.pop();
            size_t tmp_id = tmp.tid(*this);
            if (visited[tmp_id]) continue;

            visited[tmp_id] = 1;
            // std::cout << tmp_id << " ";

            m_tet_attribute[tmp_id].part_id = current_id;

            auto f_tmp1 = tmp;
            auto f_tmp2 = tmp.switch_face(*this);
            auto f_tmp3 = tmp.switch_edge(*this).switch_face(*this);
            auto f_tmp4 = tmp.switch_vertex(*this).switch_edge(*this).switch_face(*this);

            if (!m_face_attribute[f_tmp1.fid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp1.switch_tetrahedron(*this);
                if (oppo_t.has_value()) {
                    if (!visited[(*oppo_t).tid(*this)]) bfs_queue.push(*oppo_t);
                }
            }
            if (!m_face_attribute[f_tmp2.fid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp2.switch_tetrahedron(*this);
                if (oppo_t.has_value()) {
                    if (!visited[(*oppo_t).tid(*this)]) bfs_queue.push(*oppo_t);
                }
            }
            if (!m_face_attribute[f_tmp3.fid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp3.switch_tetrahedron(*this);
                if (oppo_t.has_value()) {
                    if (!visited[(*oppo_t).tid(*this)]) bfs_queue.push(*oppo_t);
                }
            }
            if (!m_face_attribute[f_tmp4.fid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp4.switch_tetrahedron(*this);
                if (oppo_t.has_value()) {
                    if (!visited[(*oppo_t).tid(*this)]) bfs_queue.push(*oppo_t);
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

    MatrixXd V(vert_capacity(), 3);
    MatrixXi T(tet_capacity(), 4);
    MatrixXi F(faces.size(), 3);

    V.setZero();
    T.setZero();
    F.setZero();

    MatrixXd parts(tet_capacity(), 1);
    parts.setZero();
    MatrixXd wn_input(tet_capacity(), 1);
    wn_input.setZero();
    MatrixXd wn_tracked(tet_capacity(), 1);
    wn_tracked.setZero();
    MatrixXd t_energy(tet_capacity(), 1);
    t_energy.setZero();
    std::vector<VectorXd> wn_per_input;
    if (!tets.empty()) {
        wn_per_input.resize(m_tet_attribute[tets[0].tid(*this)].m_winding_number_per_input.size());
        for (VectorXd& wn : wn_per_input) {
            wn.resize(tet_capacity());
        }
    }

    VectorXd v_sizing_field(vert_capacity());
    v_sizing_field.setZero();
    VectorXd v_is_rounded(vert_capacity());
    v_is_rounded.setZero();
    VectorXd v_is_on_surface(vert_capacity());
    v_is_on_surface.setZero();
    VectorXd v_is_on_open_boundary(vert_capacity());
    v_is_on_open_boundary.setZero();
    VectorXd v_order(vert_capacity());
    v_order.setZero();

    int index = 0;
    for (const Tuple& t : tets) {
        size_t tid = t.tid(*this);
        parts(index, 0) = m_tet_attribute[tid].part_id;
        wn_input(index, 0) = m_tet_attribute[tid].m_winding_number_input;
        wn_tracked(index, 0) = m_tet_attribute[tid].m_winding_number_tracked;
        t_energy(index, 0) = std::cbrt(m_tet_attribute[tid].m_quality);

        for (size_t i = 0; i < wn_per_input.size(); ++i) {
            wn_per_input[i][index] = m_tet_attribute[tid].m_winding_number_per_input[i];
        }

        const auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            T(index, j) = (int)vs[j].vid(*this);
        }
        ++index;
    }

    for (size_t i = 0; i < faces.size(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            F(i, j) = (int)faces[i][j];
        }
    }

    for (const auto& v : vs) {
        const auto vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
        v_sizing_field[vid] = m_vertex_attribute[vid].m_sizing_scalar;
        v_is_rounded[vid] = m_vertex_attribute[vid].m_is_rounded ? 1 : 0;
        v_is_on_surface[vid] = m_vertex_attribute[vid].m_is_on_surface ? 1 : 0;
        v_is_on_open_boundary[vid] = m_vertex_extra[vid].m_is_on_open_boundary ? 1 : 0;
        v_order[vid] = m_vertex_attribute[vid].m_order;
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    if (use_hdf5) {
        log_and_throw_error("Cannot write HDF5");
        // writer = std::make_shared<paraviewo::HDF5VTUWriter>();
    } else {
        writer = std::make_shared<paraviewo::VTUWriter>();
    }

    const auto out_path = path + (use_hdf5 ? ".hdf" : ".vtu");

    writer->add_cell_field("part", parts);
    writer->add_cell_field("winding_number_input", wn_input);
    writer->add_cell_field("winding_number_tracked", wn_tracked);
    writer->add_cell_field("t_energy", t_energy);
    for (size_t i = 0; i < wn_per_input.size(); ++i) {
        // Named after the input when the caller supplied input_names, as in triwild's MSH
        // groups; otherwise numbered.
        const std::string wn_name = i < m_input_names.size()
                                        ? fmt::format("wn_{}", m_input_names[i])
                                        : fmt::format("wn_{}", i);
        writer->add_cell_field(wn_name, wn_per_input[i]);
    }
    writer->add_field("sizing_field", v_sizing_field);
    writer->add_field("is_rounded", v_is_rounded);
    writer->add_field("on_surface", v_is_on_surface);
    writer->add_field("on_open_boundary", v_is_on_open_boundary);
    writer->add_field("order", v_order);


    logger().info("Write {}", out_path);
    writer->write_mesh(out_path, V, T, paraviewo::CellType::Tetrahedron);

    // surface
    {
        const auto surf_out_path = path + "_surf.vtu";
        std::shared_ptr<paraviewo::ParaviewWriter> surf_writer;
        surf_writer = std::make_shared<paraviewo::VTUWriter>();
        surf_writer->add_field("sizing_field", v_sizing_field);
        surf_writer->add_field("is_rounded", v_is_rounded);
        surf_writer->add_field("on_surface", v_is_on_surface);
        surf_writer->add_field("on_open_boundary", v_is_on_open_boundary);
        surf_writer->add_field("order", v_order);

        logger().info("Write {}", surf_out_path);
        surf_writer->write_mesh(surf_out_path, V, F, paraviewo::CellType::Triangle);
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
            visited[vid] = 1;
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

TetWildMesh::ExportStruct TetWildMesh::export_mesh_data() const
{
    ExportStruct e;

    const auto vs = get_vertices();
    const auto tets = get_tets();
    const auto faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });

    e.V.resize(vert_capacity(), 3);
    e.T.resize(tet_capacity(), 4);
    e.F.resize(faces.size(), 3);

    e.V.setZero();
    e.T.setZero();
    e.F.setZero();

    e.t_part.resize(tet_capacity(), 1);
    e.t_part.setZero();
    e.t_winding_number_input.resize(tet_capacity(), 1);
    e.t_winding_number_input.setZero();
    e.t_winding_number_tracked.resize(tet_capacity(), 1);
    e.t_winding_number_tracked.setZero();
    e.t_amips.resize(tet_capacity(), 1);
    e.t_amips.setZero();
    if (!tets.empty()) {
        e.t_winding_number_per_input.resize(
            tet_capacity(),
            m_tet_attribute[tets[0].tid(*this)].m_winding_number_per_input.size());
    }

    int index = 0;
    for (const Tuple& t : tets) {
        size_t tid = t.tid(*this);
        e.t_part(index, 0) = m_tet_attribute[tid].part_id;
        e.t_winding_number_input(index, 0) = m_tet_attribute[tid].m_winding_number_input;
        e.t_winding_number_tracked(index, 0) = m_tet_attribute[tid].m_winding_number_tracked;
        e.t_amips(index, 0) = std::cbrt(m_tet_attribute[tid].m_quality);

        for (size_t i = 0; i < (size_t)e.t_winding_number_per_input.cols(); ++i) {
            e.t_winding_number_per_input(index, i) =
                m_tet_attribute[tid].m_winding_number_per_input[i];
        }

        const auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            e.T(index, j) = vs[j].vid(*this);
        }
        ++index;
    }

    for (size_t i = 0; i < faces.size(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            e.F(i, j) = (int)faces[i][j];
        }
    }

    for (const auto& v : vs) {
        const auto vid = v.vid(*this);
        e.V.row(vid) = m_vertex_attribute[vid].m_posf;
    }

    return e;
}

} // namespace wmtk::components::tetwild
