
#include "SimWildMesh.h"

#include "wmtk/utils/Rational.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <spdlog/fmt/ostr.h>
#include <spdlog/fmt/bundled/format.h>
#include <wmtk/utils/predicates.hpp>
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

void SimWildMesh::set_quality_field(const nlohmann::json& quality_field_json)
{
    if (!quality_field_json.is_array()) {
        log_and_throw_error(
            "quality_field should be an array of objects, each defining a region and its "
            "target "
            "quality.");
    }

    for (const auto& region_json : quality_field_json) {
        if (!region_json.contains("tags")) {
            log_and_throw_error("Each quality_field entry must contain a 'tags' field.");
        }
        if (!region_json.contains("quality")) {
            log_and_throw_error("Each quality_field entry must contain a 'quality' field.");
        }
        const std::string tags_str_set = region_json["tags"];
        auto& [expr, quality] = m_quality_field.emplace_back();
        expr = expression_parser::parse(tags_str_set, m_tag_name_to_id);

        quality = region_json["quality"];

        logger().info("Added quality field: expr = {}, quality = {}", expr->to_string(), quality);
    }
}

double SimWildMesh::target_quality(const size_t tid) const
{
    double quality = m_params.stop_energy; // default
    for (const auto& [expr, q] : m_quality_field) {
        if (expr->eval(m_tet_attribute[tid].tags)) {
            quality = q;
        }
    }
    return quality;
}

double SimWildMesh::target_quality(const Tuple& t) const
{
    const auto tid = t.tid(*this);
    return target_quality(tid);
}

double SimWildMesh::quality_rel(const size_t tid) const
{
    return std::cbrt(m_tet_attribute[tid].m_quality) / target_quality(tid);
}

double SimWildMesh::quality_rel(const Tuple& t) const
{
    return quality_rel(t.tid(*this));
}

std::tuple<double, double> SimWildMesh::optimization_quality_stats()
{
    double max_quality = -1.;
    double avg_quality = 0.;
    size_t count = 0;
    for (size_t tid = 0; tid < tet_capacity(); ++tid) {
        if (!tuple_from_tet(tid).is_valid(*this)) continue;
        const double quality = quality_rel(tid);
        max_quality = std::max(max_quality, quality);
        avg_quality += quality;
        ++count;
    }
    if (count > 0) avg_quality /= count;
    return {max_quality, avg_quality};
}

std::vector<size_t> SimWildMesh::active_vertices() const
{
    // SimWild's quality target is per tag. Normalize each cell by its own target before
    // applying the shared margin; using TetOptimizerMesh's single global threshold silently
    // skips cells whose tag requests a stricter target.
    return utils::active_vertices(
        vert_capacity(),
        tet_capacity(),
        [this](size_t tid) { return tuple_from_tet(tid).is_valid(*this); },
        [this](size_t tid) { return quality_rel(tid); },
        [this](size_t tid) { return oriented_tet_vids(tid); },
        m_params.skip_good_regions_margin,
        [this](size_t vid) { return m_vertex_attribute[vid].m_is_on_surface; });
}

bool SimWildMesh::check_mesh_quality(double& max_rel_quality, const bool verbose) const
{
    bool all_good = true;
    size_t num_bad = 0;
    size_t num_total = 0;
    max_rel_quality = 0;
    for (int i = 0; i < tet_capacity(); i++) {
        const Tuple tup = tuple_from_tet(i);
        if (!tup.is_valid(*this)) {
            continue;
        }
        num_total++;
        double rel_quality = quality_rel(i);
        max_rel_quality = std::max(max_rel_quality, rel_quality);
        if (rel_quality > 1.0) {
            all_good = false;
            num_bad++;
        }
    }
    if (verbose) {
        logger().info(
            "Bad elements: {} of {}, max relative quality: {:.6}",
            num_bad,
            num_total,
            max_rel_quality);
    }
    return all_good;
}


size_t SimWildMesh::refine_sizing_around_worst(double)
{
    const int n_rings = std::max(0, m_params.stuck_refine_rings);

    // m_quality stores AMIPS^3, so the energy the "max energy" refers to is its cube root.
    const auto worst = utils::select_worst_cells(
        tet_capacity(),
        [this](size_t tid) { return tuple_from_tet(tid).is_valid(*this); },
        [this](size_t tid) {
            const double target = target_quality(tid);
            return std::cbrt(m_tet_attribute[tid].m_quality) / target; // relative quality
        },
        1.0,
        m_params.stuck_refine_num_worst);

    if (worst.empty()) {
        return 0;
    }

    // If force-split is on, record the LONGEST edge of each worst tet. split_all_edges
    // force-splits exactly those edges (bypasses the length gate), so a stuck sliver's
    // long edge is split immediately -- WITHOUT changing the sizing field.
    m_force_split_edges.clear();
    size_t already_at_size = 0;
    if (m_params.stuck_refine_force_split) {
        for (const auto& [_, tid] : worst) {
            const auto e = utils::longest_edge(
                oriented_tet_vids(tid),
                [this](size_t vid) -> const Vector3d& { return m_vertex_attribute[vid].m_posf; });
            if (m_params.stuck_refine_force_split_oversized_only) {
                // Match TetWild's ratchet guard: AMIPS is scale invariant, so splitting an
                // already-small bad tet cannot improve its shape and only drives its edge
                // length toward zero on repeated stalls.
                const auto& ev = e.vertices();
                const double sizing = (m_vertex_attribute[ev[0]].m_sizing_scalar +
                                       m_vertex_attribute[ev[1]].m_sizing_scalar) /
                                      2;
                const double len2 =
                    (m_vertex_attribute[ev[0]].m_posf - m_vertex_attribute[ev[1]].m_posf)
                        .squaredNorm();
                if (len2 <= m_params.splitting_l2 * sizing * sizing) {
                    ++already_at_size;
                    continue;
                }
            }
            m_force_split_edges.insert(e);
        }
    }
    if (already_at_size > 0) {
        logger().info(
            "[force-split] {} worst tets are already at target size; refinement cannot "
            "improve their shape",
            already_at_size);
    }

    // Seed the region with the worst tets' vertices, then BFS n_rings hops.
    std::vector<size_t> seeds;
    seeds.reserve(4 * worst.size());
    for (const auto& [_, tid] : worst) {
        for (const size_t v : oriented_tet_vids(tid)) {
            seeds.push_back(v);
        }
    }
    const auto region = utils::grow_vertex_region(seeds, n_rings, [this](size_t v) {
        return get_one_ring_vids_for_vertex_adj(v);
    });

    // Apply the multiplicative refinement, clamped at the floor.
    const auto refined = utils::apply_sizing_refinement(
        region,
        m_params.stuck_refine_factor,
        m_params.stuck_refine_min_scalar,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; });

    // Grade the refined region into its surroundings (monotone, only lowers).
    gradation_smooth_sizing(m_params.stuck_refine_gradation, refined);

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

    logger().info(
        "[stuck-refine] worst {} tets (relE {:.4}), refined {} of {} region vertices",
        worst.size(),
        worst.back().first,
        refined.size(),
        region.size());
    return refined.size();
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
    // A vertex whose exact position has no double representation. V.row(vid) below is its
    // rounding, which is NOT where the vertex is -- so an inverted-looking tet in ParaView is
    // expected here and nowhere else.
    VectorXd v_is_rounded(vert_capacity());
    v_is_rounded.setZero();

    std::vector<MatrixXd> tags(m_tags_count, MatrixXd(tet_capacity(), 1));
    VectorXd amips(tet_capacity());
    VectorXd amips_target(tet_capacity());
    VectorXd amips_rel(tet_capacity());

    int index = 0;
    for (const Tuple& t : tets) {
        size_t tid = t.tid(*this);
        for (size_t j = 0; j < m_tags_count; ++j) {
            tags[j](index, 0) = m_tet_attribute[tid].tags.count(j) ? 1 : 0;
        }
        amips[index] = std::cbrt(m_tet_attribute[tid].m_quality);
        amips_target[index] = target_quality(tid);
        amips_rel[index] = quality_rel(tid);

        const auto& tv = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            T(index, j) = tv[j].vid(*this);
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
        v_is_rounded[vid] = m_vertex_attribute[vid].m_is_rounded ? 1 : 0;
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
    writer.add_cell_field("quality_target", amips_target);
    writer.add_cell_field("quality_rel", amips_rel);
    writer.add_field("sizing_field", v_sizing_field);
    writer.add_field("vid", v_id);
    writer.add_field("is_rounded", v_is_rounded);
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
