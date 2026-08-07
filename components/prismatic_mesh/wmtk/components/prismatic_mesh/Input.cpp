#include "Types.hpp"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>

#include <Eigen/Geometry>

#include <fstream>
#include <limits>
#include <unordered_map>

namespace wmtk::components::prismatic_mesh {
namespace {

using EntityKey = std::pair<int, int>;

template <typename Entities>
void collect_entity_groups(
    const int dim,
    const Entities& entities,
    const std::map<std::pair<int, int>, std::string>& physical_names,
    std::map<EntityKey, std::set<std::string>>& result)
{
    for (const auto& entity : entities) {
        auto& names = result[{dim, entity.tag}];
        for (const int physical_tag : entity.physical_group_tags) {
            const auto it = physical_names.find({dim, physical_tag});
            if (it != physical_names.end()) {
                names.insert(it->second);
            }
        }
        // Files written by older WMTK versions often use identical entity and
        // physical tags without populating the entity association.
        if (names.empty()) {
            const auto it = physical_names.find({dim, entity.tag});
            if (it != physical_names.end()) {
                names.insert(it->second);
            }
        }
    }
}

std::set<std::string> groups_for_block(
    const int dim,
    const int entity_tag,
    const std::map<EntityKey, std::set<std::string>>& entity_groups,
    const std::map<std::pair<int, int>, std::string>& physical_names)
{
    const auto it = entity_groups.find({dim, entity_tag});
    if (it != entity_groups.end() && !it->second.empty()) {
        return it->second;
    }
    const auto ph = physical_names.find({dim, entity_tag});
    if (ph != physical_names.end()) {
        return {ph->second};
    }
    return {"__all__"};
}

size_t nodes_per_element(const int type)
{
    switch (type) {
    case 1: return 2;
    case 2: return 3;
    case 4: return 4;
    case 6: return 6;
    case 7: return 5;
    default: return 0;
    }
}

template <size_t N>
std::array<size_t, N> read_element(
    const mshio::ElementBlock& block,
    const size_t element_index,
    const std::unordered_map<size_t, size_t>& node_index)
{
    std::array<size_t, N> result;
    const size_t stride = N + 1;
    for (size_t j = 0; j < N; ++j) {
        const size_t node_tag = block.data[element_index * stride + 1 + j];
        const auto it = node_index.find(node_tag);
        if (it == node_index.end()) {
            log_and_throw_error(
                "Element in entity ({}, {}) references missing node tag {}.",
                block.entity_dim,
                block.entity_tag,
                node_tag);
        }
        result[j] = it->second;
    }
    return result;
}

double signed_tet_volume6(const MatrixXd& V, const std::array<size_t, 4>& t)
{
    const Vector3d a = V.row(t[0]);
    const Vector3d b = V.row(t[1]);
    const Vector3d c = V.row(t[2]);
    const Vector3d d = V.row(t[3]);
    return (b - a).dot((c - a).cross(d - a));
}

} // namespace

PrismaticMeshInput read_prismatic_msh(const std::filesystem::path& path)
{
    if (!std::filesystem::exists(path)) {
        log_and_throw_error("Prismatic mesh input '{}' does not exist.", path.string());
    }

    MshData data;
    data.load(path.string());
    const auto& spec = data.m_spec;

    std::map<std::pair<int, int>, std::string> physical_names;
    for (const auto& group : spec.physical_groups) {
        physical_names[{group.dim, group.tag}] = group.name;
    }

    std::map<EntityKey, std::set<std::string>> entity_groups;
    collect_entity_groups(1, spec.entities.curves, physical_names, entity_groups);
    collect_entity_groups(2, spec.entities.surfaces, physical_names, entity_groups);
    collect_entity_groups(3, spec.entities.volumes, physical_names, entity_groups);

    std::vector<Vector3d> points;
    std::unordered_map<size_t, size_t> node_index;
    for (const auto& block : spec.nodes.entity_blocks) {
        for (size_t i = 0; i < block.num_nodes_in_block; ++i) {
            const size_t tag = block.tags[i];
            const Vector3d point(block.data[3 * i], block.data[3 * i + 1], block.data[3 * i + 2]);
            const auto [it, inserted] = node_index.emplace(tag, points.size());
            if (inserted) {
                points.push_back(point);
            } else if ((points[it->second] - point).norm() > 1e-12) {
                log_and_throw_error("Node tag {} has inconsistent coordinates.", tag);
            }
        }
    }
    if (points.empty()) {
        log_and_throw_error("Input '{}' contains no vertices.", path.string());
    }

    PrismaticMeshInput result;
    result.vertices.resize(points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        result.vertices.row(i) = points[i];
    }

    for (const auto& block : spec.elements.entity_blocks) {
        const size_t node_count = nodes_per_element(block.element_type);
        if (node_count == 0 || block.num_elements_in_block == 0) {
            continue;
        }
        const auto groups =
            groups_for_block(block.entity_dim, block.entity_tag, entity_groups, physical_names);

        if (block.entity_dim == 3 && block.element_type == 4) {
            for (size_t i = 0; i < block.num_elements_in_block; ++i) {
                auto tet = read_element<4>(block, i, node_index);
                if (signed_tet_volume6(result.vertices, tet) < 0) {
                    std::swap(tet[2], tet[3]);
                }
                result.tets.push_back(tet);
                result.tet_groups.push_back(groups);
            }
        } else if (block.entity_dim == 2 && block.element_type == 2) {
            for (size_t i = 0; i < block.num_elements_in_block; ++i) {
                const auto face = read_element<3>(block, i, node_index);
                for (const auto& group : groups) {
                    result.face_groups[group].push_back(face);
                }
            }
        } else if (block.entity_dim == 1 && block.element_type == 1) {
            for (size_t i = 0; i < block.num_elements_in_block; ++i) {
                const auto edge = read_element<2>(block, i, node_index);
                for (const auto& group : groups) {
                    result.edge_groups[group].push_back(edge);
                }
            }
        }
    }

    if (result.tets.empty()) {
        log_and_throw_error("Input '{}' contains no tetrahedra.", path.string());
    }
    return result;
}

void write_hybrid_msh(const std::filesystem::path& path, const HybridVolumeMesh& mesh)
{
    if (mesh.vertices.cols() != 3) {
        log_and_throw_error("Hybrid mesh vertex matrix must have three columns.");
    }

    MshData out;
    out.add_tet_vertices(mesh.vertices.rows(), [&mesh](const size_t i) {
        return Vector3d(mesh.vertices.row(i));
    });
    out.add_tets(mesh.tets.size(), [&mesh](const size_t i) { return mesh.tets[i]; });
    out.add_prisms(mesh.prisms.size(), [&mesh](const size_t i) { return mesh.prisms[i]; });
    out.add_pyramids(mesh.pyramids.size(), [&mesh](const size_t i) { return mesh.pyramids[i]; });
    out.add_physical_group("HybridVolume");

    std::map<std::string, int> region_ids;
    auto region_id = [&region_ids](const std::string& name) {
        const std::string key = name.empty() ? "unclassified" : name;
        const auto [it, inserted] =
            region_ids.emplace(key, static_cast<int>(region_ids.size()) + 1);
        return it->second;
    };

    mshio::Data cell_type;
    cell_type.header.string_tags = {"cell_type"};
    cell_type.header.real_tags = {0.0};
    cell_type.header.int_tags = {
        0,
        1,
        static_cast<int>(mesh.tets.size() + mesh.prisms.size() + mesh.pyramids.size()),
        0,
        3};

    mshio::Data region;
    region.header.string_tags = {"region_id"};
    region.header.real_tags = {0.0};
    region.header.int_tags = cell_type.header.int_tags;

    size_t tet_i = 0;
    size_t prism_i = 0;
    size_t pyramid_i = 0;
    for (const auto& block : out.m_spec.elements.entity_blocks) {
        const size_t count = block.num_elements_in_block;
        const size_t stride = nodes_per_element(block.element_type) + 1;
        for (size_t i = 0; i < count; ++i) {
            const size_t element_tag = block.data[i * stride];
            mshio::DataEntry type_entry;
            type_entry.tag = element_tag;
            type_entry.data = {static_cast<double>(block.element_type)};
            cell_type.entries.push_back(std::move(type_entry));

            std::string region_name;
            if (block.element_type == 4 && tet_i < mesh.tet_region_tags.size()) {
                region_name = mesh.tet_region_tags[tet_i++];
            } else if (block.element_type == 6 && prism_i < mesh.prism_region_tags.size()) {
                region_name = mesh.prism_region_tags[prism_i++];
            } else if (block.element_type == 7 && pyramid_i < mesh.pyramid_region_tags.size()) {
                region_name = mesh.pyramid_region_tags[pyramid_i++];
            }
            mshio::DataEntry region_entry;
            region_entry.tag = element_tag;
            region_entry.data = {static_cast<double>(region_id(region_name))};
            region.entries.push_back(std::move(region_entry));
        }
    }
    nlohmann::json region_name_to_id = nlohmann::json::object();
    for (const auto& [name, id] : region_ids) region_name_to_id[name] = id;
    region.header.string_tags.push_back(region_name_to_id.dump());
    out.m_spec.element_data.push_back(std::move(cell_type));
    out.m_spec.element_data.push_back(std::move(region));

    std::filesystem::create_directories(path.parent_path().empty() ? "." : path.parent_path());
    out.save(path.string());
}

nlohmann::json PrismaticMeshReport::to_json() const
{
    return {
        {"input_tets", input_tets},
        {"solid_tets", solid_tets},
        {"shell_prisms", shell_prisms},
        {"shell_pyramids", shell_pyramids},
        {"rod_prisms", rod_prisms},
        {"fallback_tets", fallback_tets},
        {"invalid_prism_candidates", invalid_prism_candidates},
        {"shell_shrink_iterations", shell_shrink_iterations},
        {"collision_clipped_shell_vertices", collision_clipped_shell_vertices},
        {"topological_offset_runs", topological_offset_runs},
        {"topological_offset_tets", topological_offset_tets},
        {"topological_mapped_vertices", topological_mapped_vertices},
        {"topological_target_fallbacks", topological_target_fallbacks},
        {"offset_operation_iterations", offset_operation_iterations},
        {"equal_source_collapse_attempts", equal_source_collapse_attempts},
        {"equal_source_collapses", equal_source_collapses},
        {"unlock_attempts", unlock_attempts},
        {"unlocks", unlocks},
        {"unlock_rollbacks", unlock_rollbacks},
        {"remaining_equal_source_edges", remaining_equal_source_edges},
        {"remaining_tau_2_2", remaining_tau_2_2},
        {"open_boundary_edges", open_boundary_edges},
        {"open_boundary_candidate_tets", open_boundary_candidate_tets},
        {"open_boundary_removed_tets", open_boundary_removed_tets},
        {"marked_shell_sources", marked_shell_sources},
        {"least_squares_shell_vertices", least_squares_shell_vertices},
        {"singular_shell_vertices", singular_shell_vertices},
        {"shell_smoothing_attempts", shell_smoothing_attempts},
        {"shell_smoothing_accepts", shell_smoothing_accepts},
        {"shell_smoothing_collision_rejects", shell_smoothing_collision_rejects},
        {"shell_smoothing_iterations", shell_smoothing_iterations},
        {"solid_blocked_shell_sides", solid_blocked_shell_sides},
        {"nonmanifold_shell_edges", nonmanifold_shell_edges},
        {"shell_vertex_sectors", shell_vertex_sectors},
        {"irregular_rod_vertices", irregular_rod_vertices},
        {"rod_refined_segments", rod_refined_segments},
        {"rod_refinement_max_depth", rod_refinement_max_depth},
        {"rod_joint_tets", rod_joint_tets},
        {"rod_shell_junctions", rod_shell_junctions},
        {"rod_shell_components", rod_shell_components},
        {"rod_shell_transition_tets", rod_shell_transition_tets},
        {"duplicate_cells_removed", duplicate_cells_removed},
        {"minimum_shell_scale", minimum_shell_scale},
        {"max_amips", max_amips},
        {"warnings", warnings}};
}

PrismaticMeshParameters parameters_from_json(const nlohmann::json& json)
{
    PrismaticMeshParameters result;
    result.solid_groups = json.value("solid_groups", std::vector<std::string>{});
    result.max_shell_iterations = json.value("max_shell_iterations", 100);
    result.validity_max_depth = json.value("validity_max_depth", 8);
    result.jacobian_tolerance = json.value("jacobian_tolerance", 1e-12);
    result.use_topological_offset = json.value("use_topological_offset", true);
    result.report = json.value("report", std::string{});

    for (const auto& item : json.value("shells", nlohmann::json::array())) {
        if (!item.contains("group") || !item.contains("thickness")) {
            log_and_throw_error("Each shell requires 'group' and 'thickness'.");
        }
        ShellDefinition shell;
        shell.group = item["group"].get<std::string>();
        shell.thickness = item["thickness"].get<double>();
        if (!(shell.thickness > 0) || !std::isfinite(shell.thickness)) {
            log_and_throw_error("Shell '{}' has invalid thickness.", shell.group);
        }
        result.shells.push_back(std::move(shell));
    }

    for (const auto& item : json.value("rods", nlohmann::json::array())) {
        if (!item.contains("group") || !item.contains("thickness") ||
            !item.contains("cross_section")) {
            log_and_throw_error("Each rod requires 'group', 'thickness', and 'cross_section'.");
        }
        RodDefinition rod;
        rod.group = item["group"].get<std::string>();
        rod.thickness = item["thickness"].get<double>();
        rod.subdivisions = item.value("subdivisions", 0);
        rod.circular_blend = item.value("circular_blend", 0.0);
        if (!(rod.thickness > 0) || !std::isfinite(rod.thickness)) {
            log_and_throw_error("Rod '{}' has invalid thickness.", rod.group);
        }
        if (rod.circular_blend < 0 || rod.circular_blend > 1) {
            log_and_throw_error("Rod '{}' circular_blend must be in [0, 1].", rod.group);
        }
        for (const auto& point : item["cross_section"]) {
            if (!point.is_array() || point.size() != 2) {
                log_and_throw_error("Rod '{}' cross-section points must be [x, y].", rod.group);
            }
            rod.cross_section.emplace_back(point[0].get<double>(), point[1].get<double>());
        }
        std::string reason;
        if (!is_valid_cross_section(rod.cross_section, &reason)) {
            log_and_throw_error("Rod '{}' has invalid cross-section: {}", rod.group, reason);
        }
        result.rods.push_back(std::move(rod));
    }

    if (result.shells.empty() && result.rods.empty() && result.solid_groups.empty()) {
        log_and_throw_error("At least one solid, shell, or rod group must be requested.");
    }
    return result;
}

} // namespace wmtk::components::prismatic_mesh
