#include "quality_improvement.hpp"
#include "junction_protection.hpp"
#include "quality/background_optimizer.hpp"
#include "quality/bulk.hpp"
#include "quality/prism_region.hpp"
#include "quality/region_state.hpp"
#include "rod_mesh.hpp"

#include <cmath>
#include <limits>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {
namespace q = quality;
namespace n = quality::native;

QualityImprovementOptions QualityImprovementOptions::from_json(const nlohmann::json& j)
{
    if (!j.is_object())
        throw std::invalid_argument("Quality improvement options must be an object.");
    QualityImprovementOptions o;
#define QUALITY_OPTION(name) \
    if (j.contains(#name)) j.at(#name).get_to(o.name)
    QUALITY_OPTION(cycles);
    QUALITY_OPTION(mixed_smoothing_sweeps);
    QUALITY_OPTION(volume_passes);
    QUALITY_OPTION(column_collapse_attempts);
    QUALITY_OPTION(surface_split_budget);
    QUALITY_OPTION(surface_flip_attempts);
    QUALITY_OPTION(volume_split_budget);
    QUALITY_OPTION(time_limit);
    QUALITY_OPTION(initial_sizing_factor);
    QUALITY_OPTION(quality_target);
    QUALITY_OPTION(smoothing_target);
    QUALITY_OPTION(thickness_weight);
    QUALITY_OPTION(thickness_smoothing_weight);
    QUALITY_OPTION(thickness_tolerance);
    QUALITY_OPTION(thickness_feature_angle);
    QUALITY_OPTION(topology_enabled);
#undef QUALITY_OPTION
    // Read operation budgets as signed values first: negative JSON integers must
    // not silently wrap to essentially unlimited size_t budgets.
    for (const char* name :
         {"column_collapse_attempts",
          "surface_split_budget",
          "surface_flip_attempts",
          "volume_split_budget"})
        if (j.contains(name) && j.at(name).get<int64_t>() < 0)
            throw std::invalid_argument(
                std::string("Negative quality improvement budget: ") + name);
    o.validate();
    return o;
}

nlohmann::json QualityImprovementOptions::json() const
{
    return {
        {"cycles", cycles},
        {"mixed_smoothing_sweeps", mixed_smoothing_sweeps},
        {"volume_passes", volume_passes},
        {"column_collapse_attempts", column_collapse_attempts},
        {"surface_split_budget", surface_split_budget},
        {"surface_flip_attempts", surface_flip_attempts},
        {"volume_split_budget", volume_split_budget},
        {"time_limit", time_limit},
        {"initial_sizing_factor", initial_sizing_factor},
        {"quality_target", quality_target},
        {"smoothing_target", smoothing_target},
        {"thickness_weight", thickness_weight},
        {"thickness_smoothing_weight", thickness_smoothing_weight},
        {"thickness_tolerance", thickness_tolerance},
        {"thickness_feature_angle", thickness_feature_angle},
        {"topology_enabled", topology_enabled}};
}

void QualityImprovementOptions::validate() const
{
    if (cycles < 0 || cycles > 100 || mixed_smoothing_sweeps < 0 ||
        mixed_smoothing_sweeps > 10000 || volume_passes < 0 || volume_passes > 100 ||
        !(time_limit > 0) || !std::isfinite(time_limit) || !(initial_sizing_factor > 0) ||
        !std::isfinite(initial_sizing_factor) || !(quality_target > 3) ||
        !std::isfinite(quality_target) ||
        !(smoothing_target >= 3 && smoothing_target < quality_target) ||
        !std::isfinite(smoothing_target) || thickness_weight < 0 ||
        !std::isfinite(thickness_weight) || thickness_smoothing_weight < 0 ||
        !std::isfinite(thickness_smoothing_weight) || !(thickness_tolerance > 0) ||
        !std::isfinite(thickness_tolerance) || !(thickness_feature_angle > 0) ||
        thickness_feature_angle > 90 || !std::isfinite(thickness_feature_angle))
        throw std::invalid_argument("Invalid quality improvement options.");
}

ImprovedHybridMesh improve_prismatic_mesh_quality(
    const PrismaticMeshInput& input,
    const PrismDominantMesh& hybrid,
    const QualityImprovementOptions& options)
{
    options.validate();
    if (input.vertices.rows() == 0 || hybrid.cells.empty() || !(input.target_thickness > 0) ||
        !std::isfinite(input.target_thickness))
        throw std::invalid_argument(
            "Quality improvement requires a nonempty hybrid and positive target thickness.");
    if (!input.mesh || input.vertex_tags.size() != size_t(input.vertices.rows()) ||
        input.corr_input_vertex.size() != size_t(input.vertices.rows()) ||
        input.source_vertex_ids.size() != size_t(input.vertices.rows()))
        throw std::invalid_argument("Incomplete quality improvement construction metadata.");
    q::Deadline deadline(options.time_limit);
    n::Mesh m;
    const size_t nv = input.vertices.rows();
    m.p.reserve(nv);
    for (size_t v = 0; v < nv; ++v) {
        const n::V point = input.vertices.row(v);
        if (!point.allFinite())
            throw std::invalid_argument("Nonfinite quality improvement vertex.");
        m.p.push_back(point);
    }
    m.initial = m.base = m.p;
    m.original_n = nv;
    m.input.resize(nv);
    m.outer.assign(nv, false);
    m.fixed_vertices.assign(nv, false);
    for (const auto& cell : hybrid.cells)
        if (std::any_of(cell.source_tets.begin(), cell.source_tets.end(), [&](size_t tid) {
                std::array<size_t, 4> t;
                for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(tid, j);
                return is_rod_band_tet(input, tid) ||
                       (input.rod_state && input.offset_tet_tags[tid] == 1 &&
                        junction_tet_protected(input, t));
            }))
            for (size_t v : cell.vertices) m.fixed_vertices[v] = true;
    m.correspondence = input.corr_input_vertex;
    m.thickness.assign(nv, std::numeric_limits<double>::quiet_NaN());
    m.radius.assign(nv, 0);
    m.sizing.assign(nv, 1);
    m.lo = m.hi = m.p.front();
    for (size_t v = 0; v < nv; ++v) {
        m.lo = m.lo.cwiseMin(m.p[v]);
        m.hi = m.hi.cwiseMax(m.p[v]);
        m.input[v] = input.vertex_tags.at(v) == 1;
        if (m.input[v])
            m.thickness[v] = 0;
        else if (m.correspondence.at(v) >= 0) {
            m.base[v] = m.p.at(size_t(m.correspondence[v]));
            m.thickness[v] = (m.p[v] - m.base[v]).norm();
        }
    }
    for (const auto& cell : hybrid.cells) {
        if (cell.source_tets.empty())
            throw std::invalid_argument("Missing hybrid region provenance.");
        const size_t tid = cell.source_tets.front();
        const int region = input.offset_tet_tags.at(tid) == 1 ? 2
                           : input.input_cells.at(tid) == 1   ? 1
                                                              : 3;
        const int type = cell.type == HybridCellType::Tetrahedron ? 4
                         : cell.type == HybridCellType::Prism     ? 6
                                                                  : 7;
        n::Cell c{type, region, cell.vertices};
        if (!std::isfinite(n::quality(m.p, c)))
            throw std::invalid_argument("Invalid initial hybrid cell.");
        m.cells.push_back(std::move(c));
    }
    m.rebuild();
    const auto initial_faces = q::faces_of(m.cells);
    const auto source_topology = q::topology(m, initial_faces);
    std::set<n::Key> source_boundary;
    bool box_boundary = true;
    for (const auto& [key, f] : initial_faces)
        if (f.b == size_t(-1)) {
            source_boundary.insert(key);
            bool on_box = false;
            for (int d = 0; d < 3; ++d)
                for (double bound : {m.lo[d], m.hi[d]}) {
                    bool same_plane = true;
                    for (size_t v : f.v) same_plane &= std::abs(m.p[v][d] - bound) < 1e-10;
                    on_box |= same_plane;
                }
            box_boundary &= on_box;
        }
    // Match the benchmark's box policy; a non-box domain retains its exact
    // original exterior triangulation and fixed vertices (including case 87657).
    if (box_boundary) {
        for (size_t v = 0; v < nv; ++v)
            for (int d = 0; d < 3; ++d)
                m.outer[v] |=
                    std::abs(m.p[v][d] - m.lo[d]) < 1e-10 || std::abs(m.p[v][d] - m.hi[d]) < 1e-10;
    } else {
        for (const auto& key : source_boundary)
            for (size_t j = 1; j <= key[0]; ++j) m.outer[key[j]] = true;
    }
    std::set<std::pair<int, std::vector<size_t>>> reserved_cells;
    for (const auto& c : m.cells)
        if (m.fixed(c)) {
            auto v = c.v;
            std::sort(v.begin(), v.end());
            reserved_cells.insert({c.type, std::move(v)});
        }
    for (size_t v = 0; v < nv; ++v)
        if (m.fixed(v) && !m.input[v]) {
            m.correspondence[v] = -1;
            m.thickness[v] = NAN;
        }
    q::Rules rules(m, initial_faces);
    rules.refresh_roles(m);
    std::set<n::Edge> offset_edges;
    for (const auto& face : input.mesh->get_faces()) {
        if (input.offset_face_tags.empty() ||
            input.offset_face_tags.at(face.fid(*input.mesh)) == -1)
            continue;
        const auto vertices = input.mesh->get_face_vertices(face);
        for (int j = 0; j < 3; ++j)
            offset_edges.insert(
                n::edge(vertices[j].vid(*input.mesh), vertices[(j + 1) % 3].vid(*input.mesh)));
    }
    if (offset_edges.empty())
        throw std::invalid_argument("Quality improvement requires offset edges.");
    double average = 0;
    for (const auto& e : offset_edges) average += (m.p[e[0]] - m.p[e[1]]).norm();
    average /= offset_edges.size();
    const double h = input.target_thickness,
                 length = std::max(h, average * options.initial_sizing_factor);
    const auto sizing = q::initialize_region_sizing(m, average, options.initial_sizing_factor, h);
    std::vector<n::V> normals(nv, n::V::Zero());
    if (input.optimal_normals.rows() == input.vertices.rows())
        for (size_t v = 0; v < nv; ++v) normals[v] = input.optimal_normals.row(v);
    q::MixedReference reference(m, h, length, normals);
    const double volume0 = n::volume(m), target = options.quality_target;
    const auto validate = [&]() {
        std::set<std::pair<int, std::vector<size_t>>> retained;
        for (const auto& c : m.cells)
            if (m.fixed(c)) {
                auto v = c.v;
                std::sort(v.begin(), v.end());
                retained.insert({c.type, std::move(v)});
            }
        if (retained != reserved_cells) throw std::runtime_error("reserved_rod_cells_changed");
        for (size_t v = 0; v < nv; ++v)
            if (m.fixed(v) && m.p[v] != m.initial[v])
                throw std::runtime_error("reserved_rod_vertex_moved");
        for (auto& c : m.cells)
            if (!std::isfinite(n::quality(m.p, c)))
                throw std::runtime_error("nonpositive_mixed_cell");
        const auto faces = q::faces_of(m.cells);
        if (q::topology(m, faces) != source_topology)
            throw std::runtime_error("material_topology_changed");
        std::set<n::Key> boundary;
        for (const auto& [key, f] : faces)
            if (f.b == size_t(-1)) {
                boundary.insert(key);
                for (size_t v : f.v)
                    if (!m.outer[v]) throw std::runtime_error("internal_crack");
            }
        if (!box_boundary && boundary != source_boundary)
            throw std::runtime_error("original_boundary_changed");
        for (size_t v = 0; v < m.p.size(); ++v)
            if (m.outer[v] && m.p[v] != m.initial[v])
                throw std::runtime_error("outer_boundary_moved");
        if (std::abs(n::volume(m) - volume0) > 1e-9 * std::abs(volume0))
            throw std::runtime_error("domain_volume_changed");
        if (*std::min_element(m.sizing.begin(), m.sizing.end()) * length < h * (1 - 1e-10))
            throw std::runtime_error("sizing_below_thickness");
        return rules.audit(m, faces);
    };
    auto best = m;
    auto best_rules = rules;
    auto best_reference = reference;
    const auto energy_score = [&]() {
        return q::mixed_energy_score(
            m,
            reference,
            options.smoothing_target,
            options.thickness_weight,
            options.thickness_smoothing_weight,
            options.thickness_tolerance,
            options.thickness_feature_angle);
    };
    auto best_score = energy_score();
    double best_maximum = q::mixed_score(m, target)["maximum_AMIPS"];
    nlohmann::json report = {
        {"configuration", options.json()},
        {"reserved_rod_cells", reserved_cells.size()},
        {"reserved_rod_vertices", std::count(m.fixed_vertices.begin(), m.fixed_vertices.end(), 1)},
        {"sizing_field", sizing},
        {"baseline_quality", q::quality(m)},
        {"baseline_thickness", best_score.thickness.report},
        {"baseline_energy", best_score.report},
        {"stages", nlohmann::json::array()},
        {"adaptive_refinements", nlohmann::json::array()},
        {"selected_checkpoint", {{"cycle", 0}, {"stage", -1}}},
        {"outer_boundary_policy",
         box_boundary ? "fixed_box_geometry" : "fixed_source_triangulation"},
        {"quality_definition", "Raw AMIPS ideal=3; native prism/pyramid order-4 integral"},
        {"checkpoint_selection", "minimum global maximum AMIPS"},
        {"energy_accumulation", "exact sum of floating-point contributions"},
        {"thickness_definition",
         "Relative projected height and neighbor-height differences on shell interfaces; "
         "opposite reference normals, feature creases and target-height transitions excluded"}};
    for (int cycle = 0; cycle < options.cycles && !deadline.expired(); ++cycle) {
        if (cycle)
            report["adaptive_refinements"].push_back(
                q::refine_mixed_sizing(m, target, length, h, 1));
        for (int stage = 0; stage < 4 && !deadline.expired(); ++stage) {
            if (!options.topology_enabled && (stage == 1 || stage == 2)) continue;
            auto backup = m;
            auto backup_rules = rules;
            auto backup_reference = reference;
            q::MixedEnergyScore candidate_score;
            nlohmann::json row = {
                {"cycle", cycle + 1},
                {"stage", stage},
                {"before", q::mixed_score(m, target)}};
            try {
                if (stage == 0 || stage == 3)
                    row["operations"] = q::smooth_mixed(
                        m,
                        rules,
                        reference,
                        options.mixed_smoothing_sweeps,
                        options.smoothing_target,
                        target,
                        options.thickness_weight,
                        deadline,
                        options.thickness_smoothing_weight,
                        options.thickness_tolerance,
                        options.thickness_feature_angle);
                if (stage == 1) {
                    row["operations"]["column_collapses"] = q::prism_region_collapses(
                        m,
                        rules,
                        length,
                        options.column_collapse_attempts,
                        deadline);
                    row["operations"]["surface_splits"] = q::surface_splits(
                        m,
                        rules,
                        length,
                        options.surface_split_budget,
                        deadline,
                        nullptr,
                        [&](size_t v, n::Edge parents) { reference.midpoint(m, v, parents); });
                    row["operations"]["prism_flips"] =
                        q::prism_region_flips(m, rules, options.surface_flip_attempts, deadline);
                }
                if (stage == 2)
                    row["operations"] = q::remesh_all_tets(
                        m,
                        rules,
                        length,
                        options.volume_split_budget,
                        deadline,
                        options.volume_passes);
                reference.extend(m);
                row["validation"] = validate();
                candidate_score = energy_score();
            } catch (const std::exception& e) {
                m = std::move(backup);
                rules = std::move(backup_rules);
                reference = std::move(backup_reference);
                row["rolled_back"] = e.what();
                candidate_score = energy_score();
            }
            row["after"] = q::mixed_score(m, target);
            row["energy"] = candidate_score.report;
            row["thickness"] = candidate_score.thickness.report;
            row["seconds"] = deadline.seconds();
            report["stages"].push_back(row);
            const double maximum = row["after"]["maximum_AMIPS"];
            if (maximum < best_maximum) {
                best = m;
                best_rules = rules;
                best_reference = reference;
                best_score = candidate_score;
                best_maximum = maximum;
                report["selected_checkpoint"] = {{"cycle", cycle + 1}, {"stage", stage}};
            }
            logger()
                .info("Hybrid quality cycle {} stage {}: max AMIPS {}", cycle + 1, stage, maximum);
        }
        if (q::mixed_score(m, target)["over_target"] == 0 && energy_score().thickness.target_met)
            break;
    }
    if (options.cycles > 0) {
        auto backup = m;
        try {
            report["background_relocation"] = q::relocate_background_quality(
                m,
                rules,
                reference,
                options.smoothing_target,
                options.thickness_weight);
            validate();
            const double maximum = q::mixed_score(m, target)["maximum_AMIPS"];
            const bool selected = maximum < best_maximum;
            report["background_relocation"]["selected_after_relocation"] = selected;
            if (selected) {
                best = m;
                best_rules = rules;
                best_reference = reference;
                best_score = energy_score();
                best_maximum = maximum;
                report["selected_checkpoint"] =
                    report["stages"].empty() ? nlohmann::json{{"cycle", 0}, {"stage", -1}}
                                             : nlohmann::json{
                                                   {"cycle", report["stages"].back()["cycle"]},
                                                   {"stage", report["stages"].back()["stage"]}};
                report["selected_checkpoint"]["background_relocation"] = true;
            }
        } catch (const std::exception& e) {
            m = std::move(backup);
            report["background_relocation"] = {{"status", "rolled_back"}, {"reason", e.what()}};
        }
    }
    m = std::move(best);
    rules = std::move(best_rules);
    reference = std::move(best_reference);
    report["final_validation"] = validate();
    report["final_quality"] = q::quality(m);
    report["final_all_mixed"] = q::mixed_score(m, target);
    report["initial_domain_volume"] = volume0;
    report["final_domain_volume"] = n::volume(m);
    report["final_material_topology"] = q::topology(m, q::faces_of(m.cells));
    const auto final_score = energy_score();
    report["final_energy"] = final_score.report;
    report["final_thickness"] = final_score.thickness.report;
    report["seconds"] = deadline.seconds();
    report["quality_target_met"] = report["final_all_mixed"]["over_target"] == 0;
    report["thickness_target_met"] = final_score.thickness.target_met;
    report["target_met"] =
        report["quality_target_met"].get<bool>() && report["thickness_target_met"].get<bool>();
    report["status"] =
        report["target_met"].get<bool>() ? "complete_target_met" : "complete_target_not_reached";
    report["reference_tet_provenance"] =
        "Retopologized native mesh; construction source_tet_ids are not applicable";
    ImprovedHybridMesh result;
    result.vertices.resize(m.p.size(), 3);
    result.reference_base.resize(m.p.size(), 3);
    result.reference_normals.resize(m.p.size(), 3);
    result.source_vertex_ids = input.source_vertex_ids;
    result.source_vertex_ids.resize(m.p.size(), -1);
    int64_t next_id = 0;
    for (int64_t id : input.source_vertex_ids) {
        if (id == std::numeric_limits<int64_t>::max())
            throw std::overflow_error("No source IDs available for inserted quality vertices.");
        next_id = std::max(next_id, id + 1);
    }
    result.correspondence = m.correspondence;
    for (size_t v = 0; v < nv; ++v)
        if (m.fixed(v)) result.correspondence[v] = input.corr_input_vertex[v];
    result.target_heights = reference.height;
    for (size_t v = 0; v < m.p.size(); ++v) {
        result.vertices.row(v) = m.p[v];
        result.reference_base.row(v) = reference.anchor(m, v);
        result.reference_normals.row(v) = reference.normal[v];
        if (!reference.offset(m, v)) result.target_heights[v] = 0;
        if (v >= nv) {
            if (next_id == std::numeric_limits<int64_t>::max())
                throw std::overflow_error("Quality vertex ID overflow.");
            result.source_vertex_ids[v] = next_id++;
        }
        result.original_vertex_rows.push_back(v < nv ? int64_t(v) : -1);
        result.vertex_tags.push_back(
            m.fixed(v) ? input.vertex_tags[v]
            : m.input[v]
                ? 1
                : (m.shell[v] && std::isfinite(m.thickness[v]) && m.thickness[v] > 0 ? 2 : -1));
        result.target_edge_lengths.push_back(length * m.sizing[v]);
    }
    for (const auto& c : m.cells) {
        auto vertices = c.v;
        if (c.type == 6) {
            std::swap(vertices[1], vertices[2]);
            std::swap(vertices[4], vertices[5]);
        }
        result.cells.push_back(
            {c.type == 4   ? HybridCellType::Tetrahedron
             : c.type == 6 ? HybridCellType::Prism
                           : HybridCellType::Pyramid,
             std::move(vertices),
             c.region,
             c.energy});
    }
    result.report = std::move(report);
    return result;
}
ImprovedHybridMesh finalize_staged_mixed_mesh(
    const PrismaticMeshInput& input,
    const PrismDominantMesh& hybrid,
    const ImprovedHybridMesh* shell_stage,
    bool improve_background)
{
    ImprovedHybridMesh result;
    if (shell_stage) result = *shell_stage;
    const size_t old_n = result.vertices.rows(), nv = input.vertices.rows();
    result.vertices = input.vertices;
    result.source_vertex_ids = input.source_vertex_ids;
    result.correspondence.resize(nv, -1);
    result.original_vertex_rows.resize(nv, -1);
    result.vertex_tags.resize(nv, -1);
    result.target_heights.resize(nv, 0);
    result.target_edge_lengths.resize(nv, input.rod_state->radius);
    result.reference_base.conservativeResize(nv, 3);
    result.reference_normals.conservativeResize(nv, 3);
    for (size_t v = 0; v < nv; ++v) {
        if (v >= old_n) {
            result.reference_base.row(v) = input.vertices.row(v);
            result.reference_normals.row(v).setZero();
            result.vertex_tags[v] = input.vertex_tags[v];
            result.correspondence[v] = input.corr_input_vertex[v];
        }
    }
    result.cells.clear();
    result.construction_regions.clear();
    result.rod_regions.clear();
    result.joint_one_ring.clear();
    n::Mesh actual;
    for (size_t v = 0; v < nv; ++v) actual.p.push_back(input.vertices.row(v));
    for (const auto& c : hybrid.cells) {
        const size_t tid = c.source_tets.front();
        const int region = input.offset_tet_tags[tid] == 1 ? 2
                           : input.input_cells[tid] == 1   ? 1
                                                           : 3;
        const int type = c.type == HybridCellType::Tetrahedron ? 4
                         : c.type == HybridCellType::Prism     ? 6
                                                               : 7;
        n::Cell cell{type, region, c.vertices};
        const double energy = n::quality(actual.p, cell);
        if (!std::isfinite(energy)) throw std::runtime_error("invalid final staged mixed cell");
        actual.cells.push_back(cell);
        auto vertices = c.vertices;
        if (type == 6) {
            std::swap(vertices[1], vertices[2]);
            std::swap(vertices[4], vertices[5]);
        }
        result.cells.push_back({c.type, std::move(vertices), region, energy});
        const bool rod = std::any_of(c.source_tets.begin(), c.source_tets.end(), [&](size_t t) {
            return is_rod_band_tet(input, t);
        });
        const bool joint = rod && std::any_of(c.vertices.begin(), c.vertices.end(), [&](size_t v) {
                               return input.rod_state->joints.count(v);
                           });
        result.construction_regions.push_back(region == 2 ? rod ? 2 : 1 : 0);
        result.rod_regions.push_back(!rod ? 0 : joint ? 3 : type == 6 ? 1 : type == 4 ? 2 : 4);
        result.joint_one_ring.push_back(joint ? 1 : 0);
    }
    nlohmann::json background_report;
    if (improve_background) {
        actual.initial = actual.base = actual.p;
        actual.original_n = nv;
        actual.input.resize(nv);
        actual.outer.assign(nv, false);
        actual.fixed_vertices.assign(nv, false);
        actual.correspondence.assign(nv, -1);
        actual.thickness.assign(nv, NAN);
        actual.radius.assign(nv, 0);
        actual.sizing.assign(nv, 1);
        actual.lo = actual.hi = actual.p.front();
        for (size_t v = 0; v < nv; ++v) {
            actual.input[v] = input.vertex_tags[v] == 1;
            if (actual.input[v]) actual.thickness[v] = 0;
            actual.fixed_vertices[v] = input.rod_state->fixed_shell_vertices.count(v) != 0;
            actual.lo = actual.lo.cwiseMin(actual.p[v]);
            actual.hi = actual.hi.cwiseMax(actual.p[v]);
        }
        actual.rebuild();
        const auto faces = q::faces_of(actual.cells);
        for (const auto& [key, face] : faces)
            if (face.b == size_t(-1))
                for (size_t v : face.v) actual.outer[v] = true;
        const auto original_positions = actual.p;
        try {
            q::Rules rules(actual, faces);
            const double radius = input.rod_state->radius;
            q::MixedReference reference(actual, radius, radius, {});
            background_report = q::relocate_background_quality(actual, rules, reference, 18, 0);
            for (size_t v = 0; v < nv; ++v) result.vertices.row(v) = actual.p[v];
            for (size_t i = 0; i < actual.cells.size(); ++i)
                result.cells[i].amips = actual.cells[i].energy;
        } catch (const std::exception& e) {
            actual.p = original_positions;
            for (auto& c : actual.cells) n::quality(actual.p, c);
            background_report = {{"status", "rolled_back"}, {"reason", e.what()}};
        }
    }
    const auto shell_report = result.report;
    result.report = {
        {"method", "shell_construction_quality_then_rod_geometry_and_construction"},
        {"shell_stage", shell_stage ? shell_report : nlohmann::json(nullptr)},
        {"rod_stage", input.rod_state->report},
        {"final_quality", q::quality(actual)},
        {"final_domain_volume", n::volume(actual)},
        {"final_material_topology", q::topology(actual, q::faces_of(actual.cells))},
        {"construction", hybrid.report},
        {"reference_tet_provenance",
         "Final native cells; pre-remeshing source rows are not exported"}};
    if (improve_background) result.report["background_relocation"] = std::move(background_report);
    return result;
}
} // namespace wmtk::components::prismatic_mesh
