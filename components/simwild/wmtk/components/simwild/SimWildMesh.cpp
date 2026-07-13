
#include "SimWildMesh.h"

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

#include <paraviewo/VTMWriter.hpp>
#include <paraviewo/VTUWriter.hpp>

#include <limits>

#include "expression_parser/Parser.hpp"

namespace wmtk::components::simwild {


VertexAttributes::VertexAttributes(const Vector3r& p)
{
    m_pos = p;
    m_posf = to_double(p);
}

void SimWildMesh::mesh_improvement(int max_its)
{
    if (all_rounded() && m_params.stop_at_float) {
        logger().info("===== All vertices are rounded. Stop. =====");
        return;
    }

    ////preprocessing
    // TODO: refactor to eliminate repeated partition.
    //

    compute_vertex_partition_morton();

    // write_vtu(fmt::format("debug_{}", m_debug_print_counter++));

    logger().info("========it pre========");
    local_operations({{0, 1, 0, 0}});

    ////operation loops
    bool is_hit_min_edge_length = false;
    const int M = 2;
    int m = 0;
    double pre_max_energy = 0., pre_avg_energy = 0.;
    for (int it = 0; it < max_its; it++) {
        ///ops
        logger().info("\n========it {}========", it);
        auto [max_energy, avg_energy] = local_operations({{1, 1, 1, 1}});

        ///energy check
        logger().info("max energy {} stop {}", max_energy, m_params.stop_energy);
        if (max_energy < m_params.stop_energy) {
            break;
        }
        consolidate_mesh();

        // output_faces(
        //     m_params.output_path + "after_iter" + std::to_string(it) + ".obj",
        //     [](auto& f) { return f.m_is_surface_fs; });

        // output_mesh(m_params.output_path + "after_iter" + std::to_string(it) + ".msh");

        logger().info("V = {}, T = {}", vert_capacity(), tet_capacity());

        if (all_rounded() && m_params.stop_at_float) {
            logger().info("All vertices are rounded. Stop.");
            break;
        }

        ///sizing field
        if (it > 0 && pre_max_energy - max_energy < 5e-1 &&
            (pre_avg_energy - avg_energy) / avg_energy < 0.1) {
            m++;
            if (m == M) {
                logger().info(">>>>adjust_sizing_field...");
                if (is_hit_min_edge_length) {
                    logger().warn(
                        "Adjust sizing field although min edge length was already hit. This should "
                        "not happen.");
                }
                is_hit_min_edge_length = adjust_sizing_field_serial(max_energy);
                // is_hit_min_edge_length = adjust_sizing_field(max_energy);
                logger().info(">>>>adjust_sizing_field finished...");
                m = 0;
            }
        } else {
            m = 0;
            /**
             * Update pre energies only if they are smaller than current energies. This helps to
             * adjust the sizing field in case the energy alternates between two states.
             */
            pre_max_energy = std::min(pre_max_energy, max_energy);
            pre_avg_energy = std::min(pre_avg_energy, avg_energy);
        }
        if (is_hit_min_edge_length) {
            // todo: maybe to do sth
        }
    }

    logger().info("========it post========");
    local_operations({{0, 1, 0, 0}});
}

std::tuple<double, double> SimWildMesh::local_operations(
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
        logger().info("Sanity checks done.");
    };

    sanity_checks();

    for (int i = 0; i < ops.size(); i++) {
        timer.start();
        if (i == 0) {
            for (int n = 0; n < ops[i]; n++) {
                logger().info("==splitting {}==", n);
                split_all_edges();
                logger().info(
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
                if (m_params.debug_output) {
                    write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
                }
                auto [max_energy, avg_energy] = get_max_avg_energy();
                logger().info("split max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
                sanity_checks();
                if (max_energy < m_params.stop_energy) {
                    return std::make_tuple(max_energy, avg_energy);
                }
            }
        } else if (i == 1) {
            for (int n = 0; n < ops[i]; n++) {
                logger().info("==collapsing {}==", n);
                collapse_all_edges(collapse_limit_length);
                logger().info(
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
                if (m_params.debug_output) {
                    write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
                }
                auto [max_energy, avg_energy] = get_max_avg_energy();
                logger().info("collapse max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
                sanity_checks();
                if (max_energy < m_params.stop_energy) {
                    return std::make_tuple(max_energy, avg_energy);
                }
            }
        } else if (i == 2) {
            for (int n = 0; n < ops[i]; n++) {
                logger().info("==swapping {}==", n);
                int cnt_success = 0;
                cnt_success += swap_all_edges_all();
                // cnt_success += swap_all_edges_56();
                // cnt_success += swap_all_edges_44();
                // cnt_success += swap_all_edges();
                cnt_success += swap_all_faces();
                if (m_params.debug_output) {
                    write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
                }
                auto [max_energy, avg_energy] = get_max_avg_energy();
                logger().info("swap max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
                sanity_checks();
                if (max_energy < m_params.stop_energy) {
                    return std::make_tuple(max_energy, avg_energy);
                }
            }
        } else if (i == 3) {
            logger().info("==smoothing ==");
            smooth_all_vertices(ops[i]);
            auto [max_energy, avg_energy] = get_max_avg_energy();
            logger().info("smooth max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
            if (ops[i] > 0 && max_energy < m_params.stop_energy) {
                return std::make_tuple(max_energy, avg_energy);
            }
        }
        // output_faces(fmt::format("out-op{}.obj", i), [](auto& f) { return f.m_is_surface_fs; });
    }
    // write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    energy = get_max_avg_energy();
    logger().info("max energy = {:.6}", std::get<0>(energy));
    logger().info("avg energy = {:.6}", std::get<1>(energy));
    logger().info("time = {}", timer.getElapsedTime());


    return energy;
}

CellTag wmtk::components::simwild::SimWildMesh::string_set_to_cell_tag(
    const std::set<std::string>& str_set)
{
    CellTag cell_tag;
    for (const auto& str : str_set) {
        const auto it = m_tag_name_to_id.find(str);
        if (it != m_tag_name_to_id.end()) {
            cell_tag.insert(it->second);
        } else {
            logger().warn("Tag name {} does not exist! Adding new tag.", str);
            int64_t new_id = m_tags_count++;
            m_tag_name_to_id[str] = new_id;
            m_tag_id_to_name[new_id] = str;
            cell_tag.insert(new_id);
        }
    }
    return cell_tag;
}

void SimWildMesh::set_sizing_field(const nlohmann::json& sizing_field_json)
{
    if (!sizing_field_json.is_array()) {
        log_and_throw_error(
            "sizing_field should be an array of objects, each defining a region and its target "
            "length.");
    }

    for (const auto& region_json : sizing_field_json) {
        if (!region_json.contains("tags")) {
            log_and_throw_error("Each sizing_field entry must contain a 'tags' field.");
        }
        const std::string tags_str_set = region_json["tags"];
        auto& [expr, length] = m_sizing_field.emplace_back();
        expr = expression_parser::parse(tags_str_set, m_tag_name_to_id);

        length = region_json["length"];
        double length_rel = region_json["length_rel"];
        if (length < 0 && length_rel < 0) {
            log_and_throw_error(
                "Each sizing_field entry must specify at least one of 'length' or 'length_rel'.");
        }

        if (length < 0) {
            length = length_rel * m_params.diag_l;
        }

        logger().info("Added sizing field: expr = {}, length = {}", expr->to_string(), length);
    }

    // apply sizing fields to vertices
    for (const Tuple& t : get_tets()) {
        const auto tid = t.tid(*this);
        for (const auto& [expr, length] : m_sizing_field) {
            if (!expr->eval(m_tet_attribute[tid].tags)) {
                continue;
            }
            const auto vs = oriented_tet_vids(tid);
            for (const size_t& vid : vs) {
                auto& s = m_vertex_attribute[vid].m_sizing_scalar;
                s = length / m_params.l; // overwrite previous value
            }
        }
    }
    for (const Tuple& t : get_tets()) {
        const auto tid = t.tid(*this);
        double sizing = 1.0; // default
        for (const auto& [expr, length] : m_sizing_field) {
            if (expr->eval(m_tet_attribute[tid].tags)) {
                sizing = length / m_params.l;
            }
        }
        const auto vs = oriented_tet_vids(tid);
        for (const size_t& vid : vs) {
            auto& s = m_vertex_attribute[vid].m_sizing_scalar;
            s = std::min(s, sizing);
        }
    }
}

bool SimWildMesh::adjust_sizing_field_serial(double max_energy)
{
    logger().info("#V = {}, #T = {}", vert_capacity(), tet_capacity());

    const double stop_filter_energy = m_params.stop_energy * 0.8;
    double filter_energy = std::max(max_energy / 100, stop_filter_energy);
    filter_energy = std::min(filter_energy, 100.);

    const double recover_scalar = 1.5;
    const double refine_scalar = 0.5;
    const double min_refine_scalar = m_params.l_min / m_params.l;

    // // outputs scale_multipliers
    // std::vector<double> scale_multipliers(vert_capacity(), recover_scalar);

    std::vector<Vector3d> pts;
    std::map<size_t, double> pts_scalars;
    std::queue<size_t> v_queue;

    for (int i = 0; i < tet_capacity(); i++) {
        const Tuple t = tuple_from_tet(i);
        if (!t.is_valid(*this)) {
            continue;
        }
        const size_t tid = t.tid(*this);
        if (std::cbrt(m_tet_attribute[tid].m_quality) < filter_energy) {
            continue;
        }
        const auto vs = oriented_tet_vids(t);
        Vector3d c(0, 0, 0);
        double s = 0;
        for (int j = 0; j < 4; j++) {
            c += (m_vertex_attribute[vs[j]].m_posf);
            v_queue.emplace(vs[j]);
            s = std::max(s, m_vertex_attribute[vs[j]].m_sizing_scalar);
        }
        pts_scalars[pts.size()] = s;
        pts.emplace_back(c / 4);
    }

    logger().info("filter energy = {}; Number of low quality tets {}", filter_energy, pts.size());

    // compute maximum sizing scalar for each vertex based on the sizing field
    std::vector<double> max_sizing_scalars(vert_capacity(), std::numeric_limits<double>::max());
    for (const Tuple& t : get_tets()) {
        const auto tid = t.tid(*this);
        double sizing = std::numeric_limits<double>::max();
        bool tet_has_sizing_field = false;
        for (const auto& [expr, length] : m_sizing_field) {
            if (expr->eval(m_tet_attribute[tid].tags)) {
                sizing = std::min(sizing, length / m_params.l);
                tet_has_sizing_field = true;
            }
        }
        if (!tet_has_sizing_field) {
            sizing = 1.0; // default sizing scalar
        }
        const auto vs = oriented_tet_vids(tid);
        for (const size_t& vid : vs) {
            max_sizing_scalars[vid] = std::min(max_sizing_scalars[vid], sizing);
        }
    }

    const double R = m_params.l * 1.8;

    int sum = 0;
    int adjcnt = 0;

    KNN knn(pts);

    bool is_hit_min_edge_length = false;
    /**
     * Iterate through all vertices.
     * For each vertex, find all pts in the R-ball neighborhood.
     * Compute scalar based on the distance to the point.
     * Take smallest of all computed values.
     *
     * If no neighbor, multiply by recover_scalar.
     */
    for (int i = 0; i < vert_capacity(); i++) {
        const Tuple v = tuple_from_vertex(i);
        if (!v.is_valid(*this)) {
            continue;
        }
        const size_t vid = v.vid(*this);
        const auto& pos_v = m_vertex_attribute[vid].m_posf;

        // all low quality tet centroids within R-ball of vertex
        std::vector<nanoflann::ResultItem<uint32_t, double>> matches;
        knn.r_nearest_neighbors(pos_v, R * R, matches);

        auto& v_scalar = m_vertex_attribute[vid].m_sizing_scalar;

        if (matches.empty()) {
            // if no low quality tet within R-ball, increase sizing scalar to recover from previous
            // refinement
            v_scalar = std::min(recover_scalar * v_scalar, max_sizing_scalars[vid]);
            continue;
        }

        for (const auto& [index, sq_dist] : matches) {
            const auto& pt = pts[index];
            const double dist = std::sqrt(sq_dist);
            const double R_tet = R * pts_scalars[index]; // scale R by sizing scalar of tet
            if (dist > R_tet) {
                continue;
            }
            // linear interpolate between refine_scalar and 1 based on distance
            // double u = dist / R * (1 - refine_scalar) + refine_scalar;
            double u = dist / R_tet * (1 - refine_scalar) + refine_scalar;
            double scalar = u * pts_scalars[index];
            v_scalar = std::min(v_scalar, scalar);
        }

        if (v_scalar < min_refine_scalar) {
            v_scalar = min_refine_scalar;
            is_hit_min_edge_length = true;
        }
    }

    // restrict sizing scalar according to sizing field
    for (const Tuple& t : get_tets()) {
        const auto tid = t.tid(*this);
        double sizing = std::numeric_limits<double>::max();
        for (const auto& [expr, length] : m_sizing_field) {
            if (expr->eval(m_tet_attribute[tid].tags)) {
                sizing = std::min(sizing, length / m_params.l);
            }
        }
        const auto vs = oriented_tet_vids(tid);
        for (const size_t& vid : vs) {
            auto& s = m_vertex_attribute[vid].m_sizing_scalar;
            s = std::min(s, sizing);
        }
    }

    // std::vector<bool> visited(vert_capacity(), false);
    // std::vector<size_t> cache_one_ring;
    // // size_t vid;

    // while (!v_queue.empty()) {
    //     sum++;
    //     size_t vid = v_queue.front();
    //     v_queue.pop();
    //     if (visited[vid]) {
    //         continue;
    //     }
    //     visited[vid] = true;
    //     adjcnt++;

    //     const Vector3d& pos_v = m_vertex_attribute[vid].m_posf;
    //     double sq_dist = 0.;
    //     uint32_t idx;
    //     knn.nearest_neighbor(pos_v, idx, sq_dist);

    //     const double dist = std::sqrt(sq_dist);

    //     if (dist > R) { // outside R-ball, unmark.
    //         continue;
    //     }

    //     scale_multipliers[vid] = std::min(
    //         scale_multipliers[vid],
    //         dist / R * (1 - refine_scalar) + refine_scalar); // linear interpolate

    //     const auto vids = get_one_ring_vids_for_vertex_adj(vid, cache_one_ring);
    //     for (size_t n_vid : vids) {
    //         if (visited[n_vid]) {
    //             continue;
    //         }
    //         v_queue.push(n_vid);
    //     }
    // }

    // std::cout << sum << " " << adjcnt << std::endl;

    // bool is_hit_min_edge_length = false;
    // for (int i = 0; i < vert_capacity(); i++) {
    //     const Tuple v = tuple_from_vertex(i);
    //     if (!v.is_valid(*this)) {
    //         continue;
    //     }
    //     const size_t vid = v.vid(*this);
    //     auto& v_attr = m_vertex_attribute[vid];

    //     const double new_scale = v_attr.m_sizing_scalar * scale_multipliers[vid];
    //     if (new_scale > 1)
    //         v_attr.m_sizing_scalar = 1;
    //     else if (new_scale < min_refine_scalar) {
    //         is_hit_min_edge_length = true;
    //         v_attr.m_sizing_scalar = min_refine_scalar;
    //     } else
    //         v_attr.m_sizing_scalar = new_scale;
    // }

    return is_hit_min_edge_length;
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
void SimWildMesh::output_faces(std::string file, std::function<bool(const FaceAttributes&)> cond)
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
    logger().info("Output face size {}", outface.size());
    igl::write_triangle_mesh(file, matV, matF);
}


void SimWildMesh::init_envelope(const MatrixXd& V, const MatrixXi& F, const bool use_exact)
{
    if (m_envelope) {
        log_and_throw_error("Envelope was already initialized once.");
    }
    if (V.size() == 0 || F.size() == 0) {
        log_and_throw_error("Envelope vertices and faces cannot be empty.");
    }

    assert(m_V_envelope.empty() && m_F_envelope.empty());
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
    m_envelope->use_exact = use_exact;
    m_envelope->init(m_V_envelope, m_F_envelope, m_envelope_eps);
    m_envelope_orig = m_envelope;
}

double SimWildMesh::get_length2(const Tuple& l) const
{
    SmartTuple v1(*this, l);
    SmartTuple v2 = v1.switch_vertex();
    double length =
        (m_vertex_attribute[v1.vid()].m_posf - m_vertex_attribute[v2.vid()].m_posf).squaredNorm();
    return length;
}

void SimWildMesh::write_msh(std::string file, const bool write_envelope)
{
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_tet_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        return m_vertex_attribute[i].m_posf;
    });

    const auto& tets = get_tets();

    int64_t max_tag = -1;
    for (const Tuple& t : tets) {
        const size_t tid = t.tid(*this);
        const auto& tags = m_tet_attribute[tid].tags;
        if (tags.size() == 0) {
            continue;
        }
        int64_t mt = *tags.rbegin();
        max_tag = std::max(max_tag, mt);
    }

    if (m_tags_count < max_tag + 1) {
        logger().warn(
            "Max tag is {} but m_tags_count is {}. Adjusting m_tags_count.",
            max_tag,
            m_tags_count);
        m_tags_count = max_tag + 1;
    }

    std::vector<Tuple> tets_with_tag;
    tets_with_tag.reserve(tets.size());

    auto msh_add_tets = [&]() {
        msh.add_tets(tets_with_tag.size(), [&](size_t k) {
            auto vs = oriented_tet_vertices(tets_with_tag[k]);
            std::array<size_t, 4> data;
            for (int j = 0; j < 4; j++) {
                data[j] = vs[j].vid(*this);
            }
            return data;
        });
    };

    // ambient mesh (no non-zero tags)
    for (const Tuple& t : tets) {
        const size_t tid = t.tid(*this);
        if (m_tet_attribute[tid].tags.empty()) {
            tets_with_tag.push_back(t);
        }
    }
    msh_add_tets();

    msh.add_physical_group("ambient");

    // add a group for each tag
    for (size_t tag_img = 0; tag_img < m_tags_count; ++tag_img) {
        tets_with_tag.clear();
        for (const Tuple& t : tets) {
            const size_t tid = t.tid(*this);
            if (m_tet_attribute[tid].tags.count(tag_img)) {
                tets_with_tag.push_back(t);
            }
        }

        if (tets_with_tag.empty()) {
            continue;
        }

        msh.add_empty_vertices(3);
        msh_add_tets();

        std::string group_name;
        if (m_tag_id_to_name.count(tag_img)) {
            group_name = m_tag_id_to_name[tag_img];
        } else {
            group_name = fmt::format("tag_{}", tag_img);
            while (m_tag_name_to_id.count(group_name)) {
                group_name += "_";
            }
            m_tag_name_to_id[group_name] = tag_img;
            m_tag_id_to_name[tag_img] = group_name;
            logger().warn(
                "Tag {} does not have a name. Assigning the name {}.",
                tag_img,
                group_name);
        }
        msh.add_physical_group(group_name);
    }

    if (m_envelope && write_envelope) {
        msh.add_face_vertices(m_V_envelope.size(), [this](size_t k) { return m_V_envelope[k]; });
        msh.add_faces(m_F_envelope.size(), [this](size_t k) { return m_F_envelope[k]; });
        msh.add_physical_group("EnvelopeSurface");
    }

    msh.save(file, true);
}

std::tuple<double, double> SimWildMesh::get_max_avg_energy()
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

bool SimWildMesh::is_inverted_f(const Tuple& loc) const
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

bool SimWildMesh::is_inverted(const std::array<size_t, 4>& vs) const
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

bool SimWildMesh::is_inverted(const Tuple& loc) const
{
    auto vs = oriented_tet_vids(loc);
    return is_inverted(vs);
}

bool SimWildMesh::round(const Tuple& v)
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

double SimWildMesh::get_quality(const std::array<size_t, 4>& its) const
{
    std::array<Vector3d, 4> ps;
    bool use_rational = false;
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

double SimWildMesh::get_quality(const Tuple& loc) const
{
    auto its = oriented_tet_vids(loc);
    return get_quality(its);
}


bool SimWildMesh::invariants(const std::vector<Tuple>& tets)
{
    return true;
}

std::vector<std::array<size_t, 3>> SimWildMesh::get_faces_by_condition(
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

bool SimWildMesh::all_rounded() const
{
    size_t cnt_round = 0;
    size_t cnt_verts = 0;
    for (const Tuple& t : get_vertices()) {
        if (m_vertex_attribute[t.vid(*this)].m_is_rounded) {
            cnt_round++;
        }
        cnt_verts++;
    }
    if (cnt_round < cnt_verts) {
        logger().info("rounded {}/{}", cnt_round, cnt_verts);
        return false;
    } else {
        logger().info("All rounded!", cnt_round, cnt_verts);
        return true;
    }
}

bool SimWildMesh::is_edge_on_surface(const Tuple& loc)
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


bool SimWildMesh::is_edge_on_bbox(const Tuple& loc)
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

bool SimWildMesh::check_attributes()
{
    for (auto& f : get_faces()) {
        auto fid = f.fid(*this);
        auto vs = get_face_vertices(f);

        if (m_face_attribute[fid].m_is_surface_fs) {
            if (!(m_vertex_attribute[vs[0].vid(*this)].m_is_on_surface &&
                  m_vertex_attribute[vs[1].vid(*this)].m_is_on_surface &&
                  m_vertex_attribute[vs[2].vid(*this)].m_is_on_surface)) {
                logger().critical("surface track wrong");
                return false;
            }
            bool is_out = m_envelope->is_outside(
                {{m_vertex_attribute[vs[0].vid(*this)].m_posf,
                  m_vertex_attribute[vs[1].vid(*this)].m_posf,
                  m_vertex_attribute[vs[2].vid(*this)].m_posf}});
            if (is_out) {
                logger().critical(
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
                logger().critical("bbox track wrong {}", fid);
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
                logger().critical("is_out v");
                return false;
            }
        }

        // check rounding
        if (m_vertex_attribute[i].m_is_rounded) {
            if (m_vertex_attribute[i].m_pos[0] != m_vertex_attribute[i].m_posf[0] ||
                m_vertex_attribute[i].m_pos[1] != m_vertex_attribute[i].m_posf[1] ||
                m_vertex_attribute[i].m_pos[2] != m_vertex_attribute[i].m_posf[2]) {
                logger().critical("rounding error {} rounded", i);
                return false;
            }
        } else {
            Vector3d p = to_double(m_vertex_attribute[i].m_pos);
            if (p != m_vertex_attribute[i].m_posf) {
                logger().critical("rounding error {} unrounded", i);
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
            logger().critical(
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

void SimWildMesh::write_vtu(const std::string& path)
{
    // consolidate_mesh();
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);
    const auto& vs = get_vertices();
    const auto& tets = get_tets();
    const auto faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });
    std::vector<simplex::Edge> edges;
    for (const Tuple& t : get_edges()) {
        simplex::Edge e = simplex_from_edge(t);
        if (is_order_2_edge(e.vertices())) {
            edges.push_back(e);
        }
    }

    MatrixXd V(vert_capacity(), 3);
    MatrixXi T(tet_capacity(), 4);
    MatrixXi F(faces.size(), 3);
    MatrixXi E(edges.size(), 2);

    V.setZero();
    T.setZero();
    F.setZero();
    E.setZero();

    VectorXd v_sizing_field(vert_capacity());
    v_sizing_field.setZero();
    VectorXd v_order(vert_capacity());
    v_order.setZero();
    VectorXd v_id(vert_capacity());
    v_id.setZero();

    std::vector<MatrixXd> tags(m_tags_count, MatrixXd(tet_capacity(), 1));
    Eigen::MatrixXd amips(tet_capacity(), 1);

    int index = 0;
    for (const Tuple& t : tets) {
        size_t tid = t.tid(*this);
        for (size_t j = 0; j < m_tags_count; ++j) {
            tags[j](index, 0) = m_tet_attribute[tid].tags.count(j) ? 1 : 0;
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

    for (size_t i = 0; i < edges.size(); ++i) {
        E(i, 0) = edges[i].vertices()[0];
        E(i, 1) = edges[i].vertices()[1];
    }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
        v_sizing_field[vid] = m_vertex_attribute[vid].m_sizing_scalar;
        v_order[vid] = m_vertex_attribute[vid].m_order;
        v_id[vid] = vid;
    }

    paraviewo::VTUWriter writer;

    for (size_t j = 0; j < m_tags_count; ++j) {
        if (m_tag_id_to_name.count(j)) {
            writer.add_cell_field(m_tag_id_to_name[j], tags[j]);
        } else {
            writer.add_cell_field(fmt::format("tag_{}", j), tags[j]);
        }
    }
    writer.add_cell_field("quality", amips);
    writer.add_field("sizing_field", v_sizing_field);
    writer.add_field("vid", v_id);
    writer.write_mesh(out_path, V, T, paraviewo::CellType::Tetrahedron);

    // surface
    const std::string surf_out_path = path + "_surf.vtu";
    {
        paraviewo::VTUWriter surf_writer;
        surf_writer.add_field("sizing_field", v_sizing_field);
        surf_writer.add_field("order", v_order);
        surf_writer.add_field("vid", v_id);

        logger().info("Write {}", surf_out_path);
        surf_writer.write_mesh(surf_out_path, V, F, paraviewo::CellType::Triangle);
    }
    // edges
    const std::string edge_out_path = path + "_edge.vtu";
    {
        paraviewo::VTUWriter edge_writer;
        edge_writer.add_field("sizing_field", v_sizing_field);
        edge_writer.add_field("order", v_order);
        edge_writer.add_field("vid", v_id);


        logger().info("Write {}", edge_out_path);
        edge_writer.write_mesh(edge_out_path, V, E, paraviewo::CellType::Line);
    }

    // VTM
    const std::string vtm_path = path + ".vtm";
    paraviewo::VTMWriter vtm(m_debug_print_counter);
    vtm.add_dataset("tets", "mesh", out_path);
    vtm.add_dataset("faces", "mesh", surf_out_path);
    vtm.add_dataset("edges", "mesh", edge_out_path);
    vtm.save(vtm_path);
}

void SimWildMesh::write_surface(const std::string& path) const
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

    logger().info("Output face size {}", outface.size());
}

bool SimWildMesh::vertex_is_on_surface(const size_t vid) const
{
    return m_vertex_attribute.at(vid).m_is_on_surface;
}

bool SimWildMesh::face_is_on_surface(const size_t fid) const
{
    return m_face_attribute.at(fid).m_is_surface_fs;
}

size_t SimWildMesh::get_order_of_vertex(const size_t vid) const
{
    return m_vertex_attribute.at(vid).m_order;
}

void SimWildMesh::init_vertex_order()
{
    std::array<size_t, 4> count{{0, 0, 0, 0}};

    for (const Tuple& t : get_vertices()) {
        const size_t vid = t.vid(*this);
        const size_t order = compute_vertex_order(vid);
        m_vertex_attribute[vid].m_order = order;
        count[order]++;
    }

    logger().info("Vertex order count (0,1,2,3): {}", count);
}

double SimWildMesh::tet_volume(const size_t tid) const
{
    const auto vs = oriented_tet_vids(tid);
    const Vector3d& p0 = m_vertex_attribute[vs[0]].m_posf;
    const Vector3d& p1 = m_vertex_attribute[vs[1]].m_posf;
    const Vector3d& p2 = m_vertex_attribute[vs[2]].m_posf;
    const Vector3d& p3 = m_vertex_attribute[vs[3]].m_posf;

    const Vector3d a = (p1 - p0);
    const Vector3d b = (p2 - p0);
    const Vector3d c = (p3 - p0);

    const double v = (1. / 6.) * a.cross(b).dot(c);

    return std::abs(v);
}

} // namespace wmtk::components::simwild