
#include "TriWildMesh.h"

#include "wmtk/utils/Rational.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/io/read_edge_mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/Concurrency.hpp>
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
    partition_mesh_morton();

    logger().info("========it pre========");
    local_operations({{0, 1, 0, 0}}, false);

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
        logger().info("max energy {:.6} | stop {:.6}", max_energy, m_params.stop_energy);
        if (max_energy < m_params.stop_energy) {
            break;
        }
        consolidate_mesh();

        logger().info("#V = {}, #T = {}", vert_capacity(), tri_capacity());

        auto cnt_round = 0, cnt_verts = 0;
        TriMesh::for_each_vertex([&](auto& v) {
            if (m_vertex_attribute[v.vid(*this)].m_is_rounded) cnt_round++;
            cnt_verts++;
        });
        if (cnt_round < cnt_verts) {
            logger().info("rounded {}/{}", cnt_round, cnt_verts);
        } else {
            logger().info("All rounded!", cnt_round, cnt_verts);
        }

        ///sizing field
        if (it > 0 && pre_max_energy - max_energy < 5e-1 &&
            (pre_avg_energy - avg_energy) / avg_energy < 0.1) {
            m++;
            if (m == M) {
                logger().info(">>>>adjust_sizing_field...");
                is_hit_min_edge_length = adjust_sizing_field_serial(max_energy);
                // is_hit_min_edge_length = adjust_sizing_field(max_energy);
                logger().info(">>>>adjust_sizing_field finished...");
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

    logger().info("========it post========");
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
            if (m_envelope->is_outside(std::array<Vector2d, 2>{{p0, p1}})) {
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
                logger().info("==splitting {}==", n);
                split_all_edges();
                logger().info(
                    "#V = {}, #F = {} after split",
                    get_vertices().size(),
                    get_faces().size());
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            logger().info("split max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 1) {
            for (int n = 0; n < ops[i]; n++) {
                logger().info("==collapsing {}==", n);
                collapse_all_edges(collapse_limit_length);
                logger().info(
                    "#V = {}, #F = {} after collapse",
                    get_vertices().size(),
                    get_faces().size());
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            logger().info("collapse max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 2) {
            for (int n = 0; n < ops[i]; n++) {
                logger().info("==swapping {}==", n);
                size_t cnt_success = swap_all_edges();
                if (cnt_success == 0) {
                    break;
                }
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            logger().info("swap max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 3) {
            logger().info("==smoothing ==");
            smooth_all_vertices(ops[i]);
            auto [max_energy, avg_energy] = get_max_avg_energy();
            logger().info("smooth max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        }
    }
    energy = get_max_avg_energy();
    logger().info("max energy = {:.6}", std::get<0>(energy));
    logger().info("avg energy = {:.6}", std::get<1>(energy));
    logger().info("time = {:.4}s", timer.getElapsedTimeInSec());


    return energy;
}

void TriWildMesh::init_mesh(
    const MatrixXd& V,
    const MatrixXi& F,
    const MatrixXi& E,
    const std::vector<std::string>& tag_names)
{
    assert(V.cols() == 2);
    assert(F.cols() == 3);
    assert(E.cols() == 2);

    init(F);

    assert(check_mesh_connectivity_validity());

    m_vertex_attribute.resize(V.rows());
    m_edge_attribute.resize(F.rows() * 3);

    for (int i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_pos = to_rational(Vector2d(V.row(i)));
        m_vertex_attribute[i].m_posf = V.row(i);
    }

    // init quality and check for inverted mesh
    bool is_mesh_inverted = false;
    for (const Tuple& t : get_faces()) {
        if (is_mesh_inverted ^ is_inverted(t)) {
            if (!is_mesh_inverted) {
                is_mesh_inverted = true;
            } else {
                log_and_throw_error("Tets with different orientations in the input!");
            }
        }
        m_face_attribute[t.fid(*this)].m_quality = get_quality(t);
    }

    if (is_mesh_inverted) {
        log_and_throw_error(
            "Input mesh is fully inverted! This should not happen... Might be a bug.");
    }

    // mark edges as on surface if they are in E
    for (int i = 0; i < E.rows(); i++) {
        std::array<size_t, 2> vids = {{(size_t)E(i, 0), (size_t)E(i, 1)}};
        const auto [e, eid] = tuple_from_edge(vids);
        if (!e.is_valid(*this)) {
            log_and_throw_error("Edge {} in E is not found in the mesh!", vids);
        }
        m_edge_attribute[eid].m_is_surface_fs = true;
        m_vertex_attribute[vids[0]].m_is_on_surface = true;
        m_vertex_attribute[vids[1]].m_is_on_surface = true;
    }

    // init envelope
    if (m_envelope) {
        log_and_throw_error("Envelope was already initialized once.");
    }
    assert(m_V_envelope.empty() && m_E_envelope.empty());

    m_V_envelope.resize(V.rows());
    for (size_t i = 0; i < m_V_envelope.size(); ++i) {
        m_V_envelope[i] = V.row(i);
    }
    m_E_envelope.resize(E.rows());
    for (size_t i = 0; i < m_E_envelope.size(); ++i) {
        m_E_envelope[i] = E.row(i);
    }

    m_envelope = std::make_shared<SampleEnvelope>();
    m_envelope->init(m_V_envelope, m_E_envelope, m_envelope_eps);

    // Sanity check: All surface edges must be inside the envelope
    {
        logger().info("Envelope sanity check");
        const auto surf_edges = get_edges_by_condition([](auto& f) { return f.m_is_surface_fs; });
        for (const auto& verts : surf_edges) {
            std::array<Vector2d, 2> pp = {
                {m_vertex_attribute[verts[0]].m_posf, m_vertex_attribute[verts[1]].m_posf}};
            if (m_envelope->is_outside(pp)) {
                log_and_throw_error("Edge {} is outside!", verts);
            }
        }
        logger().info("Envelope sanity check done");
    }

    // track bounding box
    const auto edges = get_edges();
    for (size_t i = 0; i < edges.size(); i++) {
        const auto vids = get_edge_vids(edges[i]);
        int on_bbox = -1;
        for (int k = 0; k < 2; k++) {
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_min[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_min[k]) {
                on_bbox = k * 2;
                break;
            }
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_max[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_max[k]) {
                on_bbox = k * 2 + 1;
                break;
            }
        }
        if (on_bbox < 0) {
            continue;
        }
        if (edges[i].switch_face(*this)) {
            log_and_throw_error("Boundary edge {} is not on the boundary!", vids);
        }

        const size_t eid = edges[i].eid(*this);
        m_edge_attribute[eid].m_is_bbox_fs = on_bbox;

        for (const size_t vid : vids) {
            m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
        }
    }

    for_each_vertex(
        [&](auto& v) { wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces); });

    // add tag names
    for (size_t i = 0; i < tag_names.size(); ++i) {
        m_tag_id_to_name[i] = tag_names[i];
        m_tag_name_to_id[tag_names[i]] = i;
    }

    //// rounding
    size_t cnt_round = 0;

    for (int i = 0; i < vert_capacity(); i++) {
        Tuple v = tuple_from_vertex(i);
        if (round(v)) {
            cnt_round++;
        }
    }

    if (cnt_round < vert_capacity()) {
        logger().info("Rounded {}/{}", cnt_round, vert_capacity());
    } else {
        logger().info("All rounded!", cnt_round, vert_capacity());
    }
}

bool TriWildMesh::adjust_sizing_field_serial(double max_energy)
{
    logger().info("#V {}, #F {}", vert_capacity(), tri_capacity());

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

    logger().info("filter energy {} Low Quality Tets {}", filter_energy, pts.size());

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

void TriWildMesh::write_msh_groups(std::string file, const bool write_envelope)
{
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_face_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        Vector2d p2 = m_vertex_attribute[i].m_posf;
        return Vector3d(p2[0], p2[1], 0);
    });

    const auto& faces = get_faces();

    int64_t max_tag = -1;
    for (const Tuple& t : faces) {
        const size_t fid = t.fid(*this);
        const auto& tags = m_face_attribute[fid].tags;
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

    std::vector<Tuple> faces_with_tag;
    faces_with_tag.reserve(faces.size());

    auto msh_add_faces = [&]() {
        msh.add_faces(faces_with_tag.size(), [&](size_t k) {
            auto vs = oriented_tri_vids(faces_with_tag[k]);
            std::array<size_t, 3> data;
            for (int j = 0; j < 3; j++) {
                data[j] = vs[j];
            }
            return data;
        });
    };

    // ambient mesh (no non-zero tags)
    for (const Tuple& t : faces) {
        const size_t fid = t.fid(*this);
        if (m_face_attribute[fid].tags.empty()) {
            faces_with_tag.push_back(t);
        }
    }
    msh_add_faces();

    msh.add_physical_group("ambient");


    // add a group for each tag
    for (size_t tag_img = 0; tag_img < m_tags_count; ++tag_img) {
        faces_with_tag.clear();
        for (const Tuple& t : faces) {
            const size_t fid = t.fid(*this);
            if (m_face_attribute[fid].tags.count(tag_img)) {
                faces_with_tag.push_back(t);
            }
        }

        if (faces_with_tag.empty()) {
            continue;
        }

        msh.add_empty_vertices(2);
        msh_add_faces();

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
        msh.add_edge_vertices(m_V_envelope.size(), [this](size_t k) {
            return Vector3d(m_V_envelope[k][0], m_V_envelope[k][1], 0);
        });
        msh.add_edges(m_E_envelope.size(), [this](size_t k) { return m_E_envelope[k]; });
        msh.add_physical_group("EnvelopeSurface");
    }

    logger().info("Write {}", file);
    msh.save(file, true);
}

void TriWildMesh::compute_winding_numbers(const std::vector<std::string>& input_paths)
{
    const auto& faces = get_faces();
    MatrixXd C = MatrixXd::Zero(faces.size(), 2);
    for (size_t i = 0; i < faces.size(); i++) {
        const auto vs = oriented_tri_vids(faces[i]);
        for (size_t v : vs) {
            C.row(i) += m_vertex_attribute[v].m_posf;
        }
        C.row(i) /= 3;
    }

    m_tags_count = input_paths.size();
    int64_t input_idx = 0;
    for (const std::string& input_path : input_paths) {
        MatrixXd V;
        MatrixXi E;
        io::read_edge_mesh(input_path, V, E);
        assert(V.cols() == 3);
        assert(E.cols() == 2);

        V = V.block(0, 0, V.rows(), 2).eval(); // only use x,y for winding number

        // compute winding number for V,F
        Eigen::VectorXd W;
        igl::winding_number(V, E, C, W);

        if (W.maxCoeff() <= 0.5) {
            // all removed, let's invert.
            logger().info("Correcting winding number");
            for (auto i = 0; i < E.rows(); i++) {
                auto temp = E(i, 0);
                E(i, 0) = E(i, 1);
                E(i, 1) = temp;
            }
            igl::winding_number(V, E, C, W);
        }

        if (W.maxCoeff() <= 0.5) {
            logger().warn("No winding number above 0.5 for input_path {}", input_path);
        }

        // store winding number in mesh
        for (int i = 0; i < faces.size(); ++i) {
            const size_t fid = faces[i].fid(*this);
            if (W(i) > 0.5) {
                m_face_attribute[fid].tags.insert(input_idx);
            }
        }
        ++input_idx;
    }
}

int TriWildMesh::flood_fill()
{
    int current_id = 0;
    const auto faces = get_faces();
    std::map<size_t, bool> visited;

    for (const Tuple& t : faces) {
        size_t fid = t.fid(*this);
        if (visited.find(fid) != visited.end()) {
            continue;
        }

        visited[fid] = true;
        m_face_attribute[fid].part_id = current_id;

        const Tuple f1 = t;
        const Tuple f2 = t.switch_edge(*this);
        const Tuple f3 = t.switch_vertex(*this).switch_edge(*this);

        std::queue<Tuple> bfs_queue;

        if (!m_edge_attribute[f1.eid(*this)].m_is_surface_fs) {
            auto oppo_t = f1.switch_face(*this);
            if (oppo_t.has_value()) {
                if (visited.find((*oppo_t).fid(*this)) == visited.end()) {
                    bfs_queue.push(*oppo_t);
                }
            }
        }
        if (!m_edge_attribute[f2.eid(*this)].m_is_surface_fs) {
            auto oppo_t = f2.switch_face(*this);
            if (oppo_t.has_value()) {
                if (visited.find((*oppo_t).fid(*this)) == visited.end()) {
                    bfs_queue.push(*oppo_t);
                }
            }
        }
        if (!m_edge_attribute[f3.eid(*this)].m_is_surface_fs) {
            auto oppo_t = f3.switch_face(*this);
            if (oppo_t.has_value()) {
                if (visited.find((*oppo_t).fid(*this)) == visited.end()) {
                    bfs_queue.push(*oppo_t);
                }
            }
        }

        while (!bfs_queue.empty()) {
            auto tmp = bfs_queue.front();
            bfs_queue.pop();
            size_t tmp_id = tmp.fid(*this);
            if (visited.find(tmp_id) != visited.end()) continue;

            visited[tmp_id] = true;

            m_face_attribute[tmp_id].part_id = current_id;

            const Tuple f_tmp1 = tmp;
            const Tuple f_tmp2 = tmp.switch_edge(*this);
            const Tuple f_tmp3 = tmp.switch_vertex(*this).switch_edge(*this);

            if (!m_edge_attribute[f_tmp1.eid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp1.switch_face(*this);
                if (oppo_t.has_value()) {
                    if (visited.find((*oppo_t).fid(*this)) == visited.end()) {
                        bfs_queue.push(*oppo_t);
                    }
                }
            }
            if (!m_edge_attribute[f_tmp2.eid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp2.switch_face(*this);
                if (oppo_t.has_value()) {
                    if (visited.find((*oppo_t).fid(*this)) == visited.end()) {
                        bfs_queue.push(*oppo_t);
                    }
                }
            }
            if (!m_edge_attribute[f_tmp3.eid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp3.switch_face(*this);
                if (oppo_t.has_value()) {
                    if (visited.find((*oppo_t).fid(*this)) == visited.end()) {
                        bfs_queue.push(*oppo_t);
                    }
                }
            }
        }

        current_id++;
    }
    return current_id;
}

void TriWildMesh::partition_mesh()
{
    auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
    for (size_t i = 0; i < m_vertex_partition_id.size(); i++) {
        m_vertex_attribute[i].partition_id = m_vertex_partition_id[i];
    }
}

void TriWildMesh::partition_mesh_morton()
{
    if (NUM_THREADS == 0) return;
    logger().info("Number of parts: {} by morton", NUM_THREADS);

    std::vector<Vector2d> V_v(vert_capacity());

    wmtk::parallel_for(
        wmtk::blocked_range<size_t>(0, V_v.size()),
        [&](wmtk::blocked_range<size_t> r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                V_v[i] = m_vertex_attribute[i].m_posf;
            }
        },
        NUM_THREADS);

    struct sortstruct
    {
        size_t order;
        Resorting::MortonCode64 morton;
    };

    std::vector<sortstruct> list_v;
    list_v.resize(V_v.size());
    const int multi = 1000;
    // since the morton code requires a correct scale of input vertices,
    //  we need to scale the vertices if their coordinates are out of range
    std::vector<Vector2d> V = V_v; // this is for rescaling vertices
    Vector2d vmin, vmax;
    vmin = V.front();
    vmax = V.front();

    for (size_t j = 0; j < V.size(); j++) {
        for (int i = 0; i < 2; i++) {
            vmin(i) = std::min(vmin(i), V[j](i));
            vmax(i) = std::max(vmax(i), V[j](i));
        }
    }

    Vector2d center = (vmin + vmax) / 2;

    wmtk::parallel_for(
        wmtk::blocked_range<size_t>(0, V.size()),
        [&](wmtk::blocked_range<size_t> r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                V[i] = V[i] - center;
            }
        },
        NUM_THREADS);

    Vector2d scale_point =
        vmax - center; // after placing box at origin, vmax and vmin are symetric.

    double xscale, yscale;
    xscale = fabs(scale_point[0]);
    yscale = fabs(scale_point[1]);
    double scale = std::max(xscale, yscale);
    if (scale > 300) {
        wmtk::parallel_for(
            wmtk::blocked_range<size_t>(0, V.size()),
            [&](wmtk::blocked_range<size_t> r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    V[i] = V[i] / scale;
                }
            },
            NUM_THREADS);
    }

    wmtk::parallel_for(
        wmtk::blocked_range<size_t>(0, V.size()),
        [&](wmtk::blocked_range<size_t> r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                list_v[i].morton =
                    Resorting::MortonCode64(int(V[i][0] * multi), int(V[i][1] * multi), 0);
                list_v[i].order = i;
            }
        },
        NUM_THREADS);

    const auto morton_compare = [](const sortstruct& a, const sortstruct& b) {
        return (a.morton < b.morton);
    };

    wmtk::parallel_sort(list_v.begin(), list_v.end(), morton_compare);

    size_t interval = list_v.size() / NUM_THREADS + 1;

    wmtk::parallel_for(
        wmtk::blocked_range<size_t>(0, list_v.size()),
        [&](wmtk::blocked_range<size_t> r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                m_vertex_attribute[list_v[i].order].partition_id = i / interval;
            }
        },
        NUM_THREADS);
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
    return is_inverted_f(loc.fid(*this));
}

bool TriWildMesh::is_inverted_f(const size_t fid) const
{
    auto vs = oriented_tri_vids(fid);

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
        const Vector2r& v0 = m_vertex_attribute[vs[0]].m_pos;
        const Vector2r& v1 = m_vertex_attribute[vs[1]].m_pos;
        const Vector2r& v2 = m_vertex_attribute[vs[2]].m_pos;
        const Vector2r a = v1 - v0;
        const Vector2r b = v2 - v0;
        Rational res = a.x() * b.y() - a.y() * b.x();
        if (res > 0) {
            return false;
        } else {
            return true;
        }
    }
}

bool TriWildMesh::is_inverted(const Tuple& loc) const
{
    auto vs = oriented_tri_vids(loc);
    return is_inverted(vs);
}

bool TriWildMesh::is_inverted(const size_t fid) const
{
    auto vs = oriented_tri_vids(fid);
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

double TriWildMesh::get_quality(const size_t fid) const
{
    auto its = oriented_tri_vids(fid);
    return get_quality(its);
}

void TriWildMesh::write_vtu(const std::string& path) const
{
    // consolidate_mesh();
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);
    const auto& faces = get_faces();
    const auto edges = get_edges_by_condition([](auto& f) { return f.m_is_surface_fs; });

    Eigen::MatrixXd V(vert_capacity(), 2);
    Eigen::MatrixXi F(tri_capacity(), 3);
    Eigen::MatrixXi E(edges.size(), 2);

    V.setZero();
    F.setZero();
    E.setZero();

    Eigen::VectorXd v_sizing_field(vert_capacity());
    v_sizing_field.setZero();

    std::vector<MatrixXd> tags(m_tags_count, MatrixXd(tri_capacity(), 1));
    for (size_t j = 0; j < m_tags_count; ++j) {
        tags[j].setZero();
    }
    MatrixXd amips(tri_capacity(), 1);
    amips.setZero();
    MatrixXd flood_fill(tri_capacity(), 1);
    flood_fill.setZero();

    int index = 0;
    for (const Tuple& t : faces) {
        size_t fid = t.fid(*this);
        amips(index, 0) = m_face_attribute[fid].m_quality;
        for (size_t j = 0; j < m_tags_count; ++j) {
            tags[j](index, 0) = m_face_attribute[fid].tags.count(j) ? 1 : 0;
        }
        flood_fill(index, 0) = m_face_attribute[fid].part_id;

        const auto vs = oriented_tri_vids(t);
        for (size_t j = 0; j < 3; j++) {
            F(index, j) = (int)vs[j];
        }
        ++index;
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = 0; j < 2; ++j) {
            E(i, j) = (int)edges[i][j];
        }
    }

    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
        v_sizing_field[vid] = m_vertex_attribute[vid].m_sizing_scalar;
    }

    paraviewo::VTUWriter writer;

    for (size_t j = 0; j < m_tags_count; ++j) {
        writer.add_cell_field(fmt::format("tag_{}", j), tags[j]);
    }
    writer.add_cell_field("quality", amips);
    writer.add_cell_field("flood_fill", flood_fill);
    writer.add_field("sizing_field", v_sizing_field);
    writer.write_mesh(path + ".vtu", V, F, paraviewo::CellType::Triangle);

    // surface
    {
        const auto surf_out_path = path + "_surf.vtu";
        paraviewo::VTUWriter surf_writer;
        surf_writer.add_field("sizing_field", v_sizing_field);

        logger().info("Write {}", surf_out_path);
        surf_writer.write_mesh(surf_out_path, V, E, paraviewo::CellType::Line);
    }
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

} // namespace wmtk::components::triwild
