#include "prism_jacobian.hpp"
#include "rod_mesh.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>

namespace wmtk::components::prismatic_mesh {
namespace {
constexpr double pi = 3.14159265358979323846;
using Tet = std::array<size_t, 4>;
void require(bool condition, const std::string& why)
{
    if (!condition) throw std::runtime_error("rod corner buffer: " + why);
}
Vector3d position(const PrismaticMeshInput& input, size_t v)
{
    return input.vertices.row(v).transpose();
}
struct Section
{
    size_t center;
    std::vector<size_t> ring;
    Eigen::Quaterniond rotation;
};
Section section(const PrismaticMeshInput& source, size_t center)
{
    const auto& f = source.rod_state->frames.at(source.rod_state->vertex_frame.at(center));
    return {center, f.ring, source.rod_state->vertex_rotations.at(center)};
}
void align_phase(Section& section, const Eigen::Quaterniond& anchor)
{
    const auto original = section.rotation;
    double best = std::numeric_limits<double>::infinity();
    size_t shift = 0;
    for (size_t k = 0; k < section.ring.size(); ++k) {
        const Eigen::Quaterniond q =
            original * Eigen::AngleAxisd(2 * pi * k / section.ring.size(), Vector3d::UnitZ());
        const double cost = anchor.angularDistance(q);
        if (cost < best) {
            best = cost;
            shift = k;
            section.rotation = q;
        }
    }
    std::rotate(section.ring.begin(), section.ring.begin() + shift, section.ring.end());
    // q and -q are the same pose. Use a common hemisphere for interpolation.
    if (section.rotation.dot(anchor) < 0) section.rotation.coeffs() *= -1;
}
nlohmann::json quaternion(const Eigen::Quaterniond& q)
{
    return {q.w(), q.x(), q.y(), q.z()};
}
struct SubdivisionFailure : std::runtime_error
{
    nlohmann::json detail;
    SubdivisionFailure(const std::string& reason, nlohmann::json data = {})
        : std::runtime_error(reason)
        , detail(std::move(data))
    {}
};
} // namespace

static RodCornerBuffer construct_corner_buffer_level(
    const PrismaticMeshInput& source,
    int64_t corner_source_vid,
    double max_frame_angle_degrees,
    int subdivision_depth)
{
    require(source.rod_state != nullptr, "missing directed rod state");
    require(
        std::isfinite(max_frame_angle_degrees) && max_frame_angle_degrees > 0 &&
            max_frame_angle_degrees <= 90,
        "frame angle must be in (0, 90] degrees");
    const auto found = std::find(
        source.source_vertex_ids.begin(),
        source.source_vertex_ids.end(),
        corner_source_vid);
    require(found != source.source_vertex_ids.end(), "unknown corner source ID");
    const size_t corner = found - source.source_vertex_ids.begin();
    const auto& state = *source.rod_state;
    require(
        source.vertex_tags[corner] == 1 && !state.protected_input.count(corner),
        "corner must be an unprotected degree-two input vertex");
    std::vector<size_t> incoming, outgoing;
    for (const auto& edge : state.directed_input_edges) {
        if (edge[1] == corner) incoming.push_back(edge[0]);
        if (edge[0] == corner) outgoing.push_back(edge[1]);
    }
    require(incoming.size() == 1 && outgoing.size() == 1, "ambiguous directed corner");
    require(
        state.vertex_frame.count(incoming[0]) && state.vertex_frame.count(outgoing[0]),
        "both neighboring centers need eligible interface frames");
    Section left = section(source, incoming[0]), right = section(source, outgoing[0]);
    const size_t degree = left.ring.size();
    require(degree >= 3 && right.ring.size() == degree, "interface degrees differ");
    const Vector3d center = position(source, corner);
    const Vector3d tin = (center - position(source, left.center)).normalized();
    const Vector3d tout = (position(source, right.center) - center).normalized();
    require(tin.allFinite() && tout.allFinite(), "coincident section centers");

    const Eigen::Quaterniond fixed = state.vertex_rotations.at(corner);
    const std::string anchor_source = "averaged_incident_edge_frames";
    align_phase(left, fixed);
    align_phase(right, fixed);
    const double max_angle = max_frame_angle_degrees * pi / 180;
    const auto initial_layers = [&](const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
        const double angle = a.angularDistance(b);
        if (angle == 0) return size_t(1);
        const double count = std::ceil(angle / max_angle);
        require(std::isfinite(count) && count <= 128, "too many requested buffer layers");
        return std::max<size_t>(1, static_cast<size_t>(count));
    };
    const size_t factor = size_t(1) << subdivision_depth;
    const size_t base_left = initial_layers(left.rotation, fixed),
                 base_right = initial_layers(fixed, right.rotation);
    if (base_left > 128 / factor || base_right > 128 / factor)
        throw SubdivisionFailure("maximum_frame_layer_budget_exceeded");
    const size_t nl = base_left * factor, nr = base_right * factor;

    RodCornerBuffer result;
    auto& input = result.input;
    auto& hybrid = result.hybrid;
    input.rod_state = std::make_shared<RodMeshState>();
    input.rod_state->radius = state.radius;
    input.rod_state->options = state.options;
    input.rod_state->options.axial_split = false;
    std::vector<Vector3d> vertices;
    int64_t next_id =
        *std::max_element(source.source_vertex_ids.begin(), source.source_vertex_ids.end());
    const auto add_vertex =
        [&](const Vector3d& point, int label, int64_t original, int64_t parent) {
            require(next_id < 9007199254740991LL, "source IDs exhausted");
            const size_t v = vertices.size();
            vertices.push_back(point);
            input.vertex_tags.push_back(label);
            input.source_vertex_ids.push_back(original < 0 ? ++next_id : original);
            input.corr_input_vertex.push_back(parent);
            input.corr_input_vid.push_back(parent < 0 ? -1 : input.source_vertex_ids[parent]);
            input.input_to_offset_vertices.emplace_back();
            if (label == 1) input.input_vertices.push_back(v);
            if (label == 2) {
                input.offset_vertices.push_back(v);
                input.input_to_offset_vertices[parent].push_back(v);
            }
            return v;
        };
    std::vector<size_t> centers;
    std::vector<std::vector<size_t>> rings;
    std::vector<Eigen::Quaterniond> rotations;
    nlohmann::json frames = nlohmann::json::array();
    for (int side = 0; side < 2; ++side) {
        const auto& outer = side == 0 ? left : right;
        const size_t layers = side == 0 ? nl : nr;
        for (size_t j = side == 0 ? 0 : 1; j <= layers; ++j) {
            const double t = static_cast<double>(j) / layers;
            const bool is_corner = side == 0 && j == layers;
            const bool is_outer = side == 0 ? j == 0 : j == layers;
            const Vector3d point =
                is_corner   ? center
                : is_outer  ? position(source, outer.center)
                : side == 0 ? ((1 - t) * position(source, outer.center) + t * center).eval()
                            : ((1 - t) * center + t * position(source, outer.center)).eval();
            const Eigen::Quaterniond q = is_corner   ? fixed
                                         : is_outer  ? outer.rotation
                                         : side == 0 ? left.rotation.slerp(t, fixed).normalized()
                                                     : fixed.slerp(t, right.rotation).normalized();
            const int64_t original = is_corner  ? corner_source_vid
                                     : is_outer ? source.source_vertex_ids[outer.center]
                                                : -1;
            const size_t v = add_vertex(point, 1, original, -1);
            centers.push_back(v);
            rotations.push_back(q);
            std::vector<size_t> ring;
            nlohmann::json vids = nlohmann::json::array();
            for (size_t k = 0; k < degree; ++k) {
                const double a = 2 * pi * k / degree;
                const Vector3d p =
                    is_outer ? position(source, outer.ring[k])
                             : (point + state.radius * (q * Vector3d(std::cos(a), std::sin(a), 0)))
                                   .eval();
                const int64_t id = is_outer ? source.source_vertex_ids[outer.ring[k]] : -1;
                ring.push_back(add_vertex(p, 2, id, v));
                vids.push_back(input.source_vertex_ids.back());
            }
            rings.push_back(std::move(ring));
            frames.push_back(
                {{"center_vid", input.source_vertex_ids[v]},
                 {"center", {point.x(), point.y(), point.z()}},
                 {"quaternion_wxyz", quaternion(q)},
                 {"ring_vids", vids},
                 {"corner_anchor", is_corner},
                 {"interface_anchor", is_outer}});
        }
    }
    input.vertices.resize(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i) input.vertices.row(i) = vertices[i].transpose();
    std::vector<Tet> tets;
    double minimum_j = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < centers.size(); ++i) {
        const size_t u = centers[i - 1], v = centers[i];
        input.rod_state->input_edges.push_back({u, v});
        input.rod_state->directed_input_edges.push_back({u, v});
        for (size_t k = 0; k < degree; ++k) {
            const size_t a = rings[i - 1][k], b = rings[i - 1][(k + 1) % degree];
            const size_t A = rings[i][k], B = rings[i][(k + 1) % degree];
            HybridCell cell;
            cell.type = HybridCellType::Prism;
            cell.vertices = {u, a, b, v, A, B};
            cell.prism_candidate = hybrid.cells.size();
            const double j = minimum_prism_jacobian(input.vertices, cell).determinant;
            if (!std::isfinite(j) || j <= 0)
                throw SubdivisionFailure(
                    "interpolated prism has nonpositive Jacobian",
                    {{"minimum_prism_det_j",
                      std::isfinite(j) ? nlohmann::json(j) : nlohmann::json(nullptr)},
                     {"interval", i - 1},
                     {"sector", k},
                     {"left_layers", nl},
                     {"right_layers", nr}});
            minimum_j = std::min(minimum_j, j);
            // Both choices retain the same center-to-ring diagonals on shared
            // radial quads. Their outer shell diagonal may differ; the background
            // reconstruction must use the actually selected reference triangles.
            const std::vector<std::vector<Tet>> splits{
                {{u, a, b, B}, {u, a, B, A}, {u, v, A, B}},
                {{b, u, a, A}, {b, u, A, B}, {v, u, B, A}}};
            const auto split = std::find_if(splits.begin(), splits.end(), [&](const auto& ts) {
                return std::all_of(ts.begin(), ts.end(), [&](const auto& t) {
                    return tet_volume_above_threshold(
                        input.vertices,
                        t,
                        state.options.min_tet_volume);
                });
            });
            if (split == splits.end())
                throw SubdivisionFailure(
                    "buffer subdivision has nonpositive or below-floor tet",
                    {{"interval", i - 1},
                     {"sector", k},
                     {"left_layers", nl},
                     {"right_layers", nr}});
            for (const auto& tet : *split) {
                cell.source_tets.push_back(tets.size());
                tets.push_back(tet);
                hybrid.source_tet_to_cell.push_back(hybrid.cells.size());
            }
            hybrid.cells.push_back(std::move(cell));
        }
    }
    input.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i)
        for (int j = 0; j < 4; ++j) input.tetrahedra(i, j) = tets[i][j];
    input.input_cells.assign(tets.size(), 0);
    input.offset_tet_tags.assign(tets.size(), 1);
    input.mesh = std::make_unique<TetMesh>();
    input.mesh->init_with_isolated_vertices(vertices.size(), tets);
    require(input.mesh->check_mesh_connectivity_validity(), "invalid subdivision connectivity");
    validate_prism_dominant_mesh(input, hybrid, state.options.min_tet_volume);
    const auto tau = analyze_rod_tau(input);
    require(
        tau["invalid_correspondence"] == 0 && tau["edge_parent_mismatch"] == 0,
        "buffer correspondence mismatch");
    double measured_angle = 0;
    for (size_t i = 1; i < rotations.size(); ++i)
        measured_angle = std::max(measured_angle, rotations[i - 1].angularDistance(rotations[i]));
    const double anchor_error = fixed.angularDistance(rotations[nl]);
    require(
        anchor_error < 1e-14 && (position(input, centers[nl]) - center).squaredNorm() == 0,
        "corner anchor changed");
    auto& report = input.rod_state->report;
    report = {
        {"mode", "isolated_corner_buffer_preview"},
        {"corner_source_vid", corner_source_vid},
        {"anchor_source", anchor_source},
        {"fixed_corner_quaternion_wxyz", quaternion(fixed)},
        {"anchor_orientation_error_radians", anchor_error},
        {"corner_position_unchanged", true},
        {"outer_interfaces_unchanged", true},
        {"left_layers", nl},
        {"right_layers", nr},
        {"radius", state.radius},
        {"degree", degree},
        {"frames", frames},
        {"maximum_frame_angle_degrees", measured_angle * 180 / pi},
        {"minimum_prism_det_j", minimum_j},
        {"tau_after", tau},
        {"background_integrated", false},
        {"source_mesh_unchanged", true}};
    report["subdivision_depth"] = subdivision_depth;
    hybrid.report = {
        {"prisms", hybrid.cells.size()},
        {"band_tetrahedra", 0},
        {"pyramids", 0},
        {"background_tetrahedra", 0},
        {"source_tetrahedra", tets.size()},
        {"output_cells", hybrid.cells.size()},
        {"minimum_output_prism_global_det_j", minimum_j},
        {"rod", report}};
    label_offset_faces(input);
    return result;
}

static RodCornerBuffer corner_tet_fallback(
    const PrismaticMeshInput& source,
    int64_t corner_source_vid)
{
    const size_t corner = std::find(
                              source.source_vertex_ids.begin(),
                              source.source_vertex_ids.end(),
                              corner_source_vid) -
                          source.source_vertex_ids.begin();
    std::set<size_t> centers{corner};
    for (const auto& edge : source.rod_state->directed_input_edges) {
        if (edge[0] == corner) centers.insert(edge[1]);
        if (edge[1] == corner) centers.insert(edge[0]);
    }
    std::vector<Tet> old_tets;
    std::set<size_t> used = centers;
    for (int i = 0; i < source.tetrahedra.rows(); ++i) {
        if (source.offset_tet_tags[i] != 1) continue;
        Tet t;
        bool keep = true;
        for (int j = 0; j < 4; ++j) {
            const size_t v = t[j] = source.tetrahedra(i, j);
            if (source.vertex_tags[v] == 1)
                keep = keep && centers.count(v);
            else if (source.vertex_tags[v] == 2)
                keep = keep && centers.count(source.corr_input_vertex[v]);
            else
                keep = false;
        }
        if (keep) {
            old_tets.push_back(t);
            used.insert(t.begin(), t.end());
        }
    }
    require(!old_tets.empty(), "missing original tet fallback");
    RodCornerBuffer result;
    auto& input = result.input;
    auto& hybrid = result.hybrid;
    std::map<size_t, size_t> rows;
    for (size_t v : used) rows[v] = rows.size();
    input.vertices.resize(used.size(), 3);
    input.input_to_offset_vertices.resize(used.size());
    for (size_t v : used) {
        const size_t w = rows.at(v);
        input.vertices.row(w) = source.vertices.row(v);
        input.vertex_tags.push_back(source.vertex_tags[v]);
        input.source_vertex_ids.push_back(source.source_vertex_ids[v]);
        input.corr_input_vid.push_back(source.corr_input_vid[v]);
        const int64_t parent = source.corr_input_vertex[v];
        input.corr_input_vertex.push_back(parent < 0 ? -1 : static_cast<int64_t>(rows.at(parent)));
        if (source.vertex_tags[v] == 1)
            input.input_vertices.push_back(w);
        else {
            input.offset_vertices.push_back(w);
            input.input_to_offset_vertices.at(rows.at(parent)).push_back(w);
        }
    }
    std::vector<Tet> tets;
    for (auto t : old_tets) {
        for (auto& v : t) v = rows.at(v);
        HybridCell cell;
        cell.type = HybridCellType::Tetrahedron;
        cell.vertices = {t[0], t[1], t[2], t[3]};
        cell.source_tets = {tets.size()};
        tets.push_back(t);
        hybrid.source_tet_to_cell.push_back(hybrid.cells.size());
        hybrid.cells.push_back(std::move(cell));
    }
    input.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i)
        for (int j = 0; j < 4; ++j) input.tetrahedra(i, j) = tets[i][j];
    input.input_cells.assign(tets.size(), 0);
    input.offset_tet_tags.assign(tets.size(), 1);
    input.mesh = std::make_unique<TetMesh>();
    input.mesh->init_with_isolated_vertices(used.size(), tets);
    input.rod_state = std::make_shared<RodMeshState>();
    input.rod_state->options = source.rod_state->options;
    input.rod_state->options.axial_split = false;
    input.rod_state->radius = source.rod_state->radius;
    input.rod_state->report = {
        {"mode", "isolated_corner_buffer_preview"},
        {"corner_source_vid", corner_source_vid},
        {"status", "tet_fallback"},
        {"corner_position_unchanged", true},
        {"outer_interfaces_unchanged", true},
        {"source_mesh_unchanged", true},
        {"background_integrated", false}};
    hybrid.report = {
        {"prisms", 0},
        {"band_tetrahedra", tets.size()},
        {"pyramids", 0},
        {"background_tetrahedra", 0},
        {"source_tetrahedra", tets.size()},
        {"output_cells", hybrid.cells.size()}};
    validate_prism_dominant_mesh(input, hybrid, 0);
    label_offset_faces(input);
    return result;
}

RodCornerBuffer construct_rod_corner_buffer(
    const PrismaticMeshInput& source,
    int64_t corner_source_vid,
    double max_frame_angle_degrees,
    int max_subdivision_depth)
{
    require(
        max_subdivision_depth >= 0 && max_subdivision_depth <= 7,
        "maximum subdivision depth must be in [0, 7]");
    nlohmann::json attempts = nlohmann::json::array();
    for (int depth = 0; depth <= max_subdivision_depth; ++depth) {
        try {
            auto result = construct_corner_buffer_level(
                source,
                corner_source_vid,
                max_frame_angle_degrees,
                depth);
            attempts.push_back({{"depth", depth}, {"valid", true}});
            auto& r = result.input.rod_state->report;
            r["status"] = "prism_buffer";
            r["max_subdivision_depth"] = max_subdivision_depth;
            r["subdivision_attempts"] = attempts;
            result.hybrid.report["rod"] = r;
            return result;
        } catch (const SubdivisionFailure& error) {
            auto record = error.detail.is_object() ? error.detail : nlohmann::json::object();
            record["depth"] = depth;
            record["valid"] = false;
            record["reason"] = error.what();
            attempts.push_back(std::move(record));
            if (std::string(error.what()) == "maximum_frame_layer_budget_exceeded") break;
        }
    }
    auto result = corner_tet_fallback(source, corner_source_vid);
    auto& r = result.input.rod_state->report;
    r["max_subdivision_depth"] = max_subdivision_depth;
    r["subdivision_attempts"] = attempts;
    r["fallback_reason"] = "subdivision_budget_exhausted";
    result.hybrid.report["rod"] = r;
    return result;
}
} // namespace wmtk::components::prismatic_mesh
