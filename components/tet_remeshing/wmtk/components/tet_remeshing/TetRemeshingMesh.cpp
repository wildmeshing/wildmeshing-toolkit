
#include "TetRemeshingMesh.h"

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

#include <paraviewo/VTUWriter.hpp>

#include <limits>

namespace {
static int debug_print_counter = 0;
}

namespace wmtk::components::tet_remeshing {


VertexAttributes::VertexAttributes(const Vector3d& p)
    : m_posf(p)
{}

void TetRemeshingMesh::mesh_improvement(int max_its)
{
    ////preprocessing
    // TODO: refactor to eliminate repeated partition.
    //

    compute_vertex_partition_morton();

    ////operation loops
    const int M = 2;
    int m = 0;
    double pre_max_energy = 0., pre_avg_energy = 0.;
    for (int it = 0; it < max_its; it++) {
        ///ops
        wmtk::logger().info("\n========it {}========", it);
        auto [max_energy, avg_energy] = local_operations({{1, 1, 1, 1}});

        ///energy check
        wmtk::logger().info("max energy {} stop {}", max_energy, m_params.stop_energy);
        // if (max_energy < m_params.stop_energy) break;
        const bool edge_length_done = is_edge_length_converged();
        const bool energy_done = is_energy_converged();
        if (edge_length_done && energy_done) {
            break;
        } else {
            logger().info("Energy converged = {}", energy_done);
            logger().info("Edge length converged = {}", edge_length_done);
        }

        consolidate_mesh();

        // output_faces(
        //     m_params.output_path + "after_iter" + std::to_string(it) + ".obj",
        //     [](auto& f) { return f.m_is_surface_fs; });

        // output_mesh(m_params.output_path + "after_iter" + std::to_string(it) + ".msh");

        wmtk::logger().info("v {} t {}", vert_capacity(), tet_capacity());

        ///sizing field
        if (it > 0 && pre_max_energy - max_energy < 5e-1 &&
            (pre_avg_energy - avg_energy) / avg_energy < 0.1) {
            m++;
            if (m == M) {
                wmtk::logger().info(">>>>adjust_sizing_field...");
                adjust_sizing_field_serial(max_energy);
                //adjust_sizing_field_for_edge_length(); // we decided not to adjust for edge length
                wmtk::logger().info(">>>>adjust_sizing_field finished...");
                m = 0;
            }
        } else {
            m = 0;
            pre_max_energy = max_energy;
            pre_avg_energy = avg_energy;
        }
    }
}

std::tuple<double, double> TetRemeshingMesh::local_operations(const std::array<int, 4>& ops)
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
            const auto& p0 = m_vertex_attribute[verts[0]].m_posf;
            const auto& p1 = m_vertex_attribute[verts[1]].m_posf;
            const auto& p2 = m_vertex_attribute[verts[2]].m_posf;
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
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("split max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            auto [mean_el, dev_el] = get_mean_dev_edge_length();
            wmtk::logger().info("split mean edge length = {:.4} dev = {:.4}", mean_el, dev_el);
            sanity_checks();
        } else if (i == 1) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==collapsing {}==", n);
                collapse_all_edges();
                wmtk::logger().info(
                    "#vertices {}, #tets {} after collapse",
                    get_vertices().size(),
                    get_tets().size());
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("collapse max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            auto [mean_el, dev_el] = get_mean_dev_edge_length();
            wmtk::logger().info("collapse mean edge length = {:.4} dev = {:.4}", mean_el, dev_el);
            sanity_checks();
        } else if (i == 2) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==swapping {}==", n);
                int cnt_success = 0;
                cnt_success += swap_all_edges_all();
                // cnt_success += swap_all_edges_56();
                // cnt_success += swap_all_edges_44();
                // cnt_success += swap_all_edges();
                cnt_success += swap_all_faces();
                if (cnt_success == 0) {
                    break;
                }
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("swap max energy = {:.4} avg = {:.4}", max_energy, avg_energy);
            auto [mean_el, dev_el] = get_mean_dev_edge_length();
            wmtk::logger().info("swap mean edge length = {:.4} dev = {:.4}", mean_el, dev_el);
            sanity_checks();
        } else if (i == 3) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==smoothing {}==", n);
                smooth_all_vertices();
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("smooth max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            auto [mean_el, dev_el] = get_mean_dev_edge_length();
            wmtk::logger().info("smooth mean edge length = {:.4} dev = {:.4}", mean_el, dev_el);
            sanity_checks();
        }
    }
    // write_vtu(fmt::format("debug_{}", debug_print_counter++));
    energy = get_max_avg_energy();
    wmtk::logger().info("max energy = {:.6}", std::get<0>(energy));
    wmtk::logger().info("avg energy = {:.6}", std::get<1>(energy));
    wmtk::logger().info("time = {}", timer.getElapsedTime());


    return energy;
}

bool TetRemeshingMesh::adjust_sizing_field_serial(double max_energy)
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

bool TetRemeshingMesh::adjust_sizing_field_for_edge_length()
{
    /**
     * We decided that we won't adjust the sizing field in this remeshing application, and therefore
     * this code should not be used. I left it in here anyway in case it might be useful later on.
     */
    log_and_throw_error("Calling deprecated function adjust_sizing_field_for_edge_length()");
    const double edge_length_convergence_factor = 3; // this should be in m_params

    std::vector<bool> visited(vert_capacity(), false);

    const auto edges = get_edges();

    const double min_refine_scalar = m_params.l_min / m_params.l;


    /**
     * If edges are too short, it means they could not be collapsed so far. Reduce target size for
     * these edges to improve chances that all edges are within range.
     */
    for (const Tuple& t : edges) {
        const size_t v0 = t.vid(*this);
        const size_t v1 = t.switch_vertex(*this).vid(*this);

        const Vector3d& p0 = m_vertex_attribute[v0].m_posf;
        const Vector3d& p1 = m_vertex_attribute[v1].m_posf;
        const double l = (p1 - p0).norm();

        double& s0 = m_vertex_attribute[v0].m_sizing_scalar;
        double& s1 = m_vertex_attribute[v1].m_sizing_scalar;
        const double sizing_ratio = 0.5 * (s0 + s1);
        const double target_length = m_params.l * sizing_ratio;
        const double min_convergence_length =
            target_length / (0.8 * edge_length_convergence_factor);
        if (l < min_convergence_length) {
            if (!visited[v0]) {
                s0 *= 0.5;
                visited[v0] = true;
                s0 = std::max(s0, min_refine_scalar);
            }
            if (!visited[v1]) {
                s1 *= 0.5;
                visited[v1] = true;
                s1 = std::max(s1, min_refine_scalar);
            }
        }
    }
    // smooth sizing field
    for (const Tuple& t : edges) {
        const size_t v0 = t.vid(*this);
        const size_t v1 = t.switch_vertex(*this).vid(*this);

        double& s0 = m_vertex_attribute[v0].m_sizing_scalar;
        double& s1 = m_vertex_attribute[v1].m_sizing_scalar;
        // if one of the two is way larger than the other, reduce the scaling factor
        s0 = std::min(s0, 1.5 * s1);
        s1 = std::min(s1, 1.5 * s0);
    }

    return true;
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
void TetRemeshingMesh::output_faces(
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


void TetRemeshingMesh::init_envelope(const MatrixXd& V, const MatrixXi& F)
{
    if (m_envelope) {
        log_and_throw_error("Envelope was already initialized once.");
    }
    assert(m_V_envelope.empty() && m_F_envelope.empty());
    assert(V.size() != 0 && F.size() != 0);
    assert(V.cols() == 3); // vertices must be in 3D
    assert(F.cols() == 3); // envelope must be triangles


    m_V_envelope.resize(V.rows());
    for (size_t i = 0; i < m_V_envelope.size(); ++i) {
        m_V_envelope[i] = V.row(i);
    }
    m_F_envelope.resize(F.rows());
    for (size_t i = 0; i < m_F_envelope.size(); ++i) {
        m_F_envelope[i] = F.row(i);
    }

    m_envelope = std::make_shared<SampleEnvelope>();
    m_envelope->use_exact = true;
    m_envelope->init(m_V_envelope, m_F_envelope, m_envelope_eps);
}

double TetRemeshingMesh::get_length2(const Tuple& l) const
{
    SmartTuple v1(*this, l);
    SmartTuple v2 = v1.switch_vertex();
    double length =
        (m_vertex_attribute[v1.vid()].m_posf - m_vertex_attribute[v2.vid()].m_posf).squaredNorm();
    return length;
}

void TetRemeshingMesh::write_msh(std::string file)
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

    for (size_t j = 0; j < m_tags_count; ++j) {
        msh.add_tet_attribute<1>(fmt::format("tag_{}", j), [&](size_t i) {
            return m_tet_attribute[i].tags[j];
        });
    }

    msh.add_physical_group("ImageVolume");

    msh.add_face_vertices(m_V_envelope.size(), [this](size_t k) { return m_V_envelope[k]; });
    msh.add_faces(m_F_envelope.size(), [this](size_t k) { return m_F_envelope[k]; });
    msh.add_physical_group("EnvelopeSurface");

    msh.save(file, true);
}

std::tuple<double, double> TetRemeshingMesh::get_max_avg_energy()
{
    double max_energy = -1.;
    double avg_energy = 0.;
    size_t cnt = 0;

    for (int i = 0; i < tet_capacity(); i++) {
        auto tup = tuple_from_tet(i);
        if (!tup.is_valid(*this)) {
            continue;
        }

        const double& q = m_tet_attribute[tup.tid(*this)].m_quality;
        max_energy = std::max(max_energy, q);
        avg_energy += std::cbrt(q);
        cnt++;
    }

    avg_energy /= cnt;

    return std::make_tuple(std::cbrt(max_energy), avg_energy);
}

std::tuple<double, double> TetRemeshingMesh::get_mean_dev_edge_length()
{
    const auto edges = get_edges();
    std::vector<double> edge_lengths;
    edge_lengths.reserve(edges.size());

    for (const Tuple& t : edges) {
        const size_t v0 = t.vid(*this);
        const size_t v1 = t.switch_vertex(*this).vid(*this);
        const Vector3d p0 = m_vertex_attribute[v0].m_posf;
        const Vector3d p1 = m_vertex_attribute[v1].m_posf;
        const double l = (p1 - p0).norm();
        edge_lengths.emplace_back(l);
    }

    double mean = 0;
    for (const double& l : edge_lengths) {
        mean += l;
    }
    mean /= edge_lengths.size();

    double dev = 0;
    for (const double& l : edge_lengths) {
        dev += (l - mean) * (l - mean);
    }
    dev = std::sqrt(dev / edge_lengths.size());

    return std::make_tuple(mean, dev);
}

bool TetRemeshingMesh::is_edge_length_in_range()
{
    /**
     * We decided that we won't adjust the sizing field in this remeshing application, and therefore
     * this code should not be used. I left it in here anyway in case it might be useful later on.
     */
    log_and_throw_error("Calling deprecated function is_edge_length_in_range()");
    const double edge_length_convergence_factor = 3; // this should be in m_params

    const auto edges = get_edges();

    bool is_converged = true;

    size_t short_edges = 0;
    size_t long_edges = 0;

    for (const Tuple& t : edges) {
        const size_t v0 = t.vid(*this);
        const size_t v1 = t.switch_vertex(*this).vid(*this);

        const Vector3d& p0 = m_vertex_attribute[v0].m_posf;
        const Vector3d& p1 = m_vertex_attribute[v1].m_posf;
        const double l = (p1 - p0).norm();

        const double& s0 = m_vertex_attribute[v0].m_sizing_scalar;
        const double& s1 = m_vertex_attribute[v1].m_sizing_scalar;
        const double sizing_ratio = 0.5 * (s0 + s1);
        const double target_length = m_params.l * sizing_ratio;
        const double min_convergence_length = target_length / edge_length_convergence_factor;
        const double max_convergence_length = target_length * edge_length_convergence_factor;
        if (l < min_convergence_length) {
            is_converged = false;
            ++short_edges;
        }
        if (l > max_convergence_length) {
            is_converged = false;
            ++long_edges;
        }
    }

    logger().warn("Short edges ratio = {}", (double)short_edges / edges.size());
    logger().warn("Long edges ratio = {}", (double)long_edges / edges.size());

    return is_converged;
}

bool TetRemeshingMesh::is_edge_length_converged()
{
    //pre_max_energy - max_energy < 5e-1 && (pre_avg_energy - avg_energy) / avg_energy < 0.1

    double& m0 = m_edge_length_stats.mean;
    double& d0 = m_edge_length_stats.dev;

    const auto [m1, d1] = get_mean_dev_edge_length();

    const double m_change = std::abs(m0 - m1) / m0;
    const double d_change = std::abs(d0 - d1) / d0;

    // update stats
    m0 = m1;
    d0 = d1;

    logger().info("Mean change: {:.2e} | Dev change: {:.2e}", m_change, d_change);

    return (m_change < m_params.edge_length_convergence) &&
           (d_change < m_params.edge_length_convergence);
}

bool TetRemeshingMesh::is_energy_converged()
{
    const auto [max_e, avg_e] = get_max_avg_energy();
    return max_e < m_params.stop_energy;
}

bool TetRemeshingMesh::is_inverted_f(const Tuple& loc) const
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

bool TetRemeshingMesh::is_inverted(const std::array<size_t, 4>& vs) const
{
    // Return a positive value if the point pd lies below the
    // plane passing through pa, pb, and pc; "below" is defined so
    // that pa, pb, and pc appear in counterclockwise order when
    // viewed from above the plane.

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
}

bool TetRemeshingMesh::is_inverted(const Tuple& loc) const
{
    auto vs = oriented_tet_vids(loc);
    return is_inverted(vs);
}

double TetRemeshingMesh::get_quality(const std::array<size_t, 4>& its) const
{
    std::array<Vector3d, 4> ps;
    for (int k = 0; k < 4; k++) {
        ps[k] = m_vertex_attribute[its[k]].m_posf;
    }
    double energy = -1.;
    {
        std::array<double, 12> T;
        for (int k = 0; k < 4; k++)
            for (int j = 0; j < 3; j++) T[k * 3 + j] = ps[k][j];

        energy = wmtk::AMIPS_energy_stable_p3<wmtk::Rational>(T);
    }
    if (std::isinf(energy) || std::isnan(energy) || energy < 27 - 1e-3) return MAX_ENERGY;
    return energy;
}

double TetRemeshingMesh::get_quality(const Tuple& loc) const
{
    auto its = oriented_tet_vids(loc);
    return get_quality(its);
}

bool TetRemeshingMesh::invariants(const std::vector<Tuple>& tets)
{
    return true;
}

std::vector<std::array<size_t, 3>> TetRemeshingMesh::get_faces_by_condition(
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

bool TetRemeshingMesh::is_edge_on_surface(const Tuple& loc)
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


bool TetRemeshingMesh::is_edge_on_bbox(const Tuple& loc)
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

bool TetRemeshingMesh::is_vertex_on_boundary(const size_t e0)
{
    if (!m_vertex_attribute.at(e0).m_is_on_open_boundary) {
        return false;
    }

    const auto neigh_vids = get_one_ring_vids_for_vertex(e0);
    const auto e0_tids = get_one_ring_tids_for_vertex(e0);

    for (const size_t e1 : neigh_vids) {
        if (!m_vertex_attribute.at(e1).m_is_on_open_boundary) {
            continue;
        }
        int cnt = 0;
        for (int t_id : e0_tids) {
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

bool TetRemeshingMesh::check_attributes()
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
    for (const Tuple& v : vs) {
        size_t i = v.vid(*this);
        if (m_vertex_attribute[i].m_is_on_surface) {
            bool is_out = m_envelope->is_outside(m_vertex_attribute[i].m_posf);
            if (is_out) {
                wmtk::logger().critical("is_out v");
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

int TetRemeshingMesh::count_vertex_links(const Tuple& v)
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

int TetRemeshingMesh::count_edge_links(const Tuple& e)
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

int TetRemeshingMesh::flood_fill()
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

void TetRemeshingMesh::write_vtu(const std::string& path)
{
    // consolidate_mesh();
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);
    const auto& vs = get_vertices();
    const auto& tets = get_tets();
    const auto faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });

    Eigen::MatrixXd V(vert_capacity(), 3);
    Eigen::MatrixXi T(tet_capacity(), 4);
    Eigen::MatrixXi F(faces.size(), 3);

    V.setZero();
    T.setZero();
    F.setZero();

    Eigen::VectorXd v_sizing_field(vert_capacity());
    v_sizing_field.setZero();

    Eigen::MatrixXd parts(tet_capacity(), 1);
    std::vector<MatrixXd> tags(m_tags_count, MatrixXd(tet_capacity(), 1));
    Eigen::MatrixXd amips(tet_capacity(), 1);

    int index = 0;
    for (const Tuple& t : tets) {
        size_t tid = t.tid(*this);
        parts(index, 0) = m_tet_attribute[tid].part_id;
        for (size_t j = 0; j < m_tags_count; ++j) {
            tags[j](index, 0) = m_tet_attribute[tid].tags[j];
        }
        amips(index, 0) = std::cbrt(m_tet_attribute[tid].m_quality);

        const auto& vs = oriented_tet_vertices(t);
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

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
        v_sizing_field[vid] = m_vertex_attribute[vid].m_sizing_scalar;
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();

    writer->add_cell_field("part", parts);
    for (size_t j = 0; j < m_tags_count; ++j) {
        writer->add_cell_field(fmt::format("tag_{}", j), tags[j]);
    }
    writer->add_cell_field("quality", amips);
    writer->add_field("sizing_field", v_sizing_field);
    writer->write_mesh(path + ".vtu", V, T);

    // surface
    {
        const auto surf_out_path = path + "_surf.vtu";
        std::shared_ptr<paraviewo::ParaviewWriter> surf_writer;
        surf_writer = std::make_shared<paraviewo::VTUWriter>();
        surf_writer->add_field("sizing_field", v_sizing_field);

        logger().info("Write {}", surf_out_path);
        surf_writer->write_mesh(surf_out_path, V, F);
    }
}

void TetRemeshingMesh::write_surface(const std::string& path) const
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

void TetRemeshingMesh::init_sizing_field()
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

} // namespace wmtk::components::tet_remeshing