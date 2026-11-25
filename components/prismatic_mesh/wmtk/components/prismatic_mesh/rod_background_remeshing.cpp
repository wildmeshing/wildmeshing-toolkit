#include "junction_protection.hpp"
#include "quality/tet_backend.hpp"
#include "rod_mesh.hpp"

#include <igl/AABB.h>
#include <algorithm>
#include <chrono>
#include <limits>

namespace wmtk::components::prismatic_mesh {
namespace {
using Tet = std::array<size_t, 4>;
using Tri = std::array<size_t, 3>;
using Clock = std::chrono::steady_clock;

struct SurfaceDistance
{
    MatrixXd vertices;
    MatrixXi faces;
    igl::AABB<MatrixXd, 3> tree;
    double squared(const Vector3d& p) const
    {
        int face;
        Eigen::RowVector3d closest;
        return tree.squared_distance(vertices, faces, p.transpose(), face, closest);
    }
};

// The stock TetWild topology/acceptance machinery, with a fixed extracted
// boundary, a radius restriction on new/moving points and floating-point-safe
// splits. Everything is private until the full proposal has been validated.
class NeighborhoodMesh : public quality::StockFixedBackground
{
    const SurfaceDistance& surface;
    double radius_squared;
    Clock::time_point end;

public:
    NeighborhoodMesh(
        OptimizerParameters& params,
        const std::vector<Vector3d>& p,
        const std::vector<Tet>& t,
        const std::vector<uint8_t>& pins,
        const std::vector<double>& sizing,
        const SurfaceDistance& surface,
        double radius,
        Clock::time_point end)
        : StockFixedBackground(params, p, t, pins, "", sizing)
        , surface(surface)
        , radius_squared(radius * radius)
        , end(end)
    {}
    bool expired() const { return Clock::now() >= end; }
    bool inside(const Vector3d& p) const
    {
        return surface.squared(p) <= radius_squared * (1 + 1e-12);
    }
    bool split_edge_before(const Tuple& e) override
    {
        const Vector3d midpoint = (m_vertex_attribute[e.vid(*this)].m_posf +
                                   m_vertex_attribute[e.switch_vertex(*this).vid(*this)].m_posf) /
                                  2;
        return !expired() && inside(midpoint) && StockFixedBackground::split_edge_before(e);
    }
    void split_after_vertex(size_t v, bool boundary) override
    {
        if (m_vertex_attribute[v].m_is_rounded)
            StockFixedBackground::split_after_vertex(v, boundary);
    }
    bool split_edge_after(const Tuple& e) override
    {
        return TetOptimizerMesh::split_edge_after(e) &&
               m_vertex_attribute[e.vid(*this)].m_is_rounded &&
               inside(m_vertex_attribute[e.vid(*this)].m_posf);
    }
    bool swap_before_interior(const std::vector<size_t>&) override { return !expired(); }
    bool swap_edge_before(const Tuple& e) override
    {
        return !expired() && TetOptimizerMesh::swap_edge_before(e);
    }
    bool swap_face_before(const Tuple& f) override
    {
        return !expired() && TetOptimizerMesh::swap_face_before(f);
    }
    bool smooth_after(const Tuple& v) override
    {
        return TetOptimizerMesh::smooth_after(v) && inside(m_vertex_attribute[v.vid(*this)].m_posf);
    }
};
} // namespace

RodBackgroundNeighborhood rod_background_neighborhood(const PrismaticMeshInput& input)
{
    RodBackgroundNeighborhood result;
    const auto& state = *input.rod_state;
    result.radius = state.options.background_remeshing.radius > 0
                        ? state.options.background_remeshing.radius
                        : state.radius;
    std::map<Tri, size_t> front_counts;
    std::vector<std::vector<size_t>> incident(input.vertices.rows());
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        for (int j = 0; j < 4; ++j) incident[input.tetrahedra(tid, j)].push_back(tid);
        if (!is_rod_band_tet(input, tid)) continue;
        for (int omit = 0; omit < 4; ++omit) {
            Tri face;
            size_t k = 0;
            for (int j = 0; j < 4; ++j)
                if (j != omit) face[k++] = input.tetrahedra(tid, j);
            std::sort(face.begin(), face.end());
            ++front_counts[face];
        }
    }
    std::vector<Tri> front;
    for (const auto& [f, count] : front_counts)
        if (count == 1 &&
            std::all_of(f.begin(), f.end(), [&](size_t v) { return input.vertex_tags[v] == 2; }))
            front.push_back(f);
    if (front.empty()) return result;
    SurfaceDistance surface;
    surface.vertices = input.vertices;
    surface.faces.resize(front.size(), 3);
    for (size_t i = 0; i < front.size(); ++i)
        for (int j = 0; j < 3; ++j) surface.faces(i, j) = front[i][j];
    surface.tree.init(surface.vertices, surface.faces);
    std::vector<bool> near(input.vertices.rows(), false);
    for (size_t v = 0; v < near.size(); ++v)
        if (!incident[v].empty())
            near[v] =
                surface.squared(input.vertices.row(v).transpose()) <= result.radius * result.radius;
    std::map<Tri, size_t> patch_faces;
    std::set<size_t> patch_vertices, boundary;
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        if (input.input_cells[tid] == 1 || input.offset_tet_tags[tid] == 1 ||
            is_fixed_shell_tet(input, tid))
            continue;
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(tid, j);
        if (std::none_of(t.begin(), t.end(), [&](size_t v) { return near[v]; })) continue;
        ++result.selected_tets;
        patch_vertices.insert(t.begin(), t.end());
        for (int omit = 0; omit < 4; ++omit) {
            Tri f;
            size_t k = 0;
            for (int j = 0; j < 4; ++j)
                if (j != omit) f[k++] = t[j];
            std::sort(f.begin(), f.end());
            ++patch_faces[f];
        }
    }
    for (const auto& [f, count] : patch_faces)
        if (count == 1) boundary.insert(f.begin(), f.end());
    result.boundary_vertices = boundary.size();
    for (size_t v : patch_vertices)
        if (near[v] && input.vertex_tags[v] == -1 && !boundary.count(v) &&
            !state.fixed_shell_vertices.count(v) &&
            !(v < input.fixed_background_vertices.size() && input.fixed_background_vertices[v]) &&
            std::all_of(incident[v].begin(), incident[v].end(), [&](size_t tid) {
                return input.input_cells[tid] != 1 && input.offset_tet_tags[tid] != 1;
            }))
            result.movable_vertices.insert(v);
    return result;
}

void remesh_rod_background(PrismaticMeshInput& input)
{
    if (!input.rod_state || !input.rod_state->options.background_remeshing.enabled) return;
    auto& state = *input.rod_state;
    const auto& options = state.options.background_remeshing;
    const auto started = Clock::now();
    const double radius = options.radius > 0 ? options.radius : state.radius;
    const double length = options.target_length > 0 ? options.target_length : state.radius;
    if (!(std::isfinite(radius) && radius > 0 && std::isfinite(length) && length > 0))
        throw std::runtime_error("Invalid rod background neighborhood radius/length.");
    auto report = nlohmann::json{
        {"enabled", true},
        {"applied", false},
        {"radius", radius},
        {"radius_definition", "Euclidean distance outside the actual pre-prism rod surface"},
        {"selection",
         "background tets with at least one vertex within radius; outer patch faces fixed"},
        {"target_length_at_surface", length},
        {"passes_requested", options.passes},
        {"time_limit", options.time_limit},
        {"rod_positions_and_connectivity_fixed", true},
        {"rod_background_interface_fixed_during_remeshing", true},
        {"interface_swaps_run_afterward", true}};
    const auto finish = [&] {
        report["elapsed_seconds"] = std::chrono::duration<double>(Clock::now() - started).count();
        state.report["background_neighborhood_remeshing"] = report;
    };
    std::map<Tri, size_t> face_counts;
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        if (!is_rod_band_tet(input, tid)) continue;
        for (int omit = 0; omit < 4; ++omit) {
            Tri f;
            size_t k = 0;
            for (int j = 0; j < 4; ++j)
                if (j != omit) f[k++] = input.tetrahedra(tid, j);
            std::sort(f.begin(), f.end());
            ++face_counts[f];
        }
    }
    SurfaceDistance surface;
    surface.vertices = input.vertices;
    std::vector<Tri> front;
    for (const auto& [f, count] : face_counts)
        if (count == 1 &&
            std::all_of(f.begin(), f.end(), [&](size_t v) { return input.vertex_tags[v] == 2; }))
            front.push_back(f);
    if (front.empty()) {
        report["skip_reason"] = "no_rod_surface";
        finish();
        return;
    }
    surface.faces.resize(front.size(), 3);
    for (size_t i = 0; i < front.size(); ++i)
        for (int j = 0; j < 3; ++j) surface.faces(i, j) = front[i][j];
    surface.tree.init(surface.vertices, surface.faces);
    std::vector<double> distances(input.vertices.rows());
    for (size_t v = 0; v < distances.size(); ++v)
        distances[v] = std::sqrt(surface.squared(input.vertices.row(v).transpose()));
    std::vector<uint8_t> selected(input.tetrahedra.rows(), false);
    std::vector<size_t> global;
    std::map<size_t, size_t> local;
    std::vector<Tet> source;
    for (size_t tid = 0; tid < selected.size(); ++tid) {
        if (input.input_cells[tid] == 1 || input.offset_tet_tags[tid] == 1 ||
            is_fixed_shell_tet(input, tid))
            continue;
        Tet t;
        bool near = false;
        for (int j = 0; j < 4; ++j) {
            t[j] = input.tetrahedra(tid, j);
            near |= distances[t[j]] <= radius;
        }
        if (!near) continue;
        selected[tid] = true;
        for (size_t& v : t) {
            const auto [it, inserted] = local.emplace(v, local.size());
            if (inserted) global.push_back(v);
            v = it->second;
        }
        source.push_back(t);
    }
    report["selected_background_tets"] = source.size();
    report["rod_surface_triangles"] = front.size();
    if (source.empty()) {
        report["skip_reason"] = "empty_neighborhood";
        finish();
        return;
    }
    std::vector<Vector3d> points;
    std::vector<uint8_t> pins;
    std::vector<double> sizing;
    for (size_t v : global) {
        points.push_back(input.vertices.row(v));
        pins.push_back(
            input.vertex_tags[v] != -1 || distances[v] > radius ||
            (v < input.fixed_background_vertices.size() && input.fixed_background_vertices[v]) ||
            state.fixed_shell_vertices.count(v) || !junction_collapse_allowed(input, v));
        // Match the dense layer to the coarser fixed outside boundary gradually.
        sizing.push_back(1 + std::min(1., distances[v] / radius));
    }
    std::vector<Vector3d> pp;
    std::vector<int64_t> origin;
    std::vector<Tet> tt;
    try {
        OptimizerParameters params;
        params.l = length;
        params.l_min = length / 1000;
        params.diag_l =
            (input.vertices.colwise().maxCoeff() - input.vertices.colwise().minCoeff()).norm();
        params.stop_energy = 10;
        params.splitting_l2 = std::pow(1.5 * length, 2);
        params.collapsing_l2 = std::pow(2. * length / 3, 2);
        params.w_amips = 1;
        params.w_envelope = 0;
        params.skip_good_regions = false;
        const auto end = started + std::chrono::duration_cast<Clock::duration>(
                                       std::chrono::duration<double>(options.time_limit));
        NeighborhoodMesh backend(params, points, source, pins, sizing, surface, radius, end);
        backend.set_budget(options.max_splits, [&] { return backend.expired(); });
        report["quality_before"] = backend.measure();
        int completed = 0;
        for (; completed < options.passes && !backend.expired(); ++completed)
            backend.local_operations({1, 1, 1, 1});
        backend.validate_fixed_boundary();
        if (!backend.check_mesh_connectivity_validity() || !backend.round_and_check_all_rounded())
            throw std::runtime_error("invalid_neighborhood_mesh");
        backend.export_data(pp, origin, tt);
        report["backend"] = backend.report();
        report["passes_completed"] = completed;
        report["time_limit_reached"] = backend.expired();
    } catch (const std::exception& error) {
        report["rejected_proposal"] = error.what();
        finish();
        logger().warn("Rod background neighborhood unchanged: {}", error.what());
        return;
    }
    const size_t old_n = input.vertices.rows();
    MatrixXd proposed = input.vertices;
    proposed.conservativeResize(old_n + std::count(origin.begin(), origin.end(), -1), 3);
    std::vector<size_t> numbering(pp.size());
    size_t next = old_n;
    for (size_t i = 0; i < pp.size(); ++i) {
        const size_t v = origin[i] < 0 ? next++ : global.at(origin[i]);
        numbering[i] = v;
        proposed.row(v) = pp[i].transpose();
    }
    std::vector<Tet> cells;
    std::vector<int> regions, bands;
    for (size_t tid = 0; tid < selected.size(); ++tid) {
        if (selected[tid]) continue;
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(tid, j);
        cells.push_back(t);
        regions.push_back(input.input_cells[tid]);
        bands.push_back(input.offset_tet_tags[tid]);
    }
    for (auto t : tt) {
        for (size_t& v : t) v = numbering[v];
        cells.push_back(t);
        regions.push_back(0);
        bands.push_back(-1);
    }
    for (const auto& t : cells)
        if (!tet_volume_above_threshold(proposed, t, 0))
            throw std::runtime_error("rod background remeshing produced nonpositive tet");
    auto mesh = std::make_unique<TetMesh>();
    mesh->init_with_isolated_vertices(proposed.rows(), cells);
    if (!mesh->check_mesh_connectivity_validity())
        throw std::runtime_error("rod background remeshing connectivity invalid");
    int64_t next_id =
        *std::max_element(input.source_vertex_ids.begin(), input.source_vertex_ids.end());
    if (next_id >= 9007199254740991LL - int64_t(next - old_n))
        throw std::runtime_error("rod background remeshing exhausted source IDs");
    size_t moved = 0;
    for (size_t v = 0; v < old_n; ++v) {
        const bool changed = input.vertices.row(v) != proposed.row(v);
        moved += changed;
        if (changed && (input.vertex_tags[v] != -1 || distances[v] > radius))
            throw std::runtime_error("rod background remeshing moved a protected vertex");
    }
    input.vertices = std::move(proposed);
    for (size_t v = old_n; v < next; ++v) {
        input.vertex_tags.push_back(-1);
        input.source_vertex_ids.push_back(++next_id);
        input.corr_input_vid.push_back(-1);
        input.corr_input_vertex.push_back(-1);
    }
    input.input_to_offset_vertices.resize(next);
    if (!input.input_to_components.empty()) input.input_to_components.resize(next);
    if (!input.vertex_component_ids.empty()) input.vertex_component_ids.resize(next, -1);
    if (!input.singular_vertex_tags.empty()) input.singular_vertex_tags.resize(next, -1);
    if (!input.fixed_background_vertices.empty())
        input.fixed_background_vertices.resize(next, false);
    if (input.optimal_normals.rows()) {
        input.optimal_normals.conservativeResize(next, 3);
        input.optimal_normals.bottomRows(next - old_n).setZero();
    }
    if (input.target_positions.rows()) {
        input.target_positions.conservativeResize(next, 3);
        input.target_positions.bottomRows(next - old_n) = input.vertices.bottomRows(next - old_n);
    }
    input.tetrahedra.resize(cells.size(), 4);
    for (size_t i = 0; i < cells.size(); ++i)
        for (int j = 0; j < 4; ++j) input.tetrahedra(i, j) = cells[i][j];
    input.input_cells = std::move(regions);
    input.offset_tet_tags = std::move(bands);
    input.mesh = std::move(mesh);
    input.offset_face_tags.clear();
    verify_junction_one_rings(input);
    report["applied"] = true;
    report["output_neighborhood_tets"] = tt.size();
    report["new_background_vertices"] = next - old_n;
    report["moved_background_vertices"] = moved;
    finish();
    logger().info(
        "Rod background remeshing: R={}, {} -> {} tets, {} new points, {} moved points",
        radius,
        source.size(),
        tt.size(),
        next - old_n,
        moved);
}
} // namespace wmtk::components::prismatic_mesh
