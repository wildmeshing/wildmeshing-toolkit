#include "MeshReduction.hpp"

#include "NumpyCompat.hpp"

#include <wmtk/utils/Logger.hpp>

#include <mshio/mshio.h>

#include <algorithm>
#include <cmath>
#include <filesystem>

namespace wmtk::components::polyfem_ops {

namespace {

/// One .msh, read the way both mirrored Python functions read it: gmsh's node list in file order
/// and the cells of every physical group of the mesh's own dimension, group by group (ascending
/// tag, as `gmsh.model.getPhysicalGroups` hands them back), entity by entity (ascending tag, as
/// `gmsh.model.getEntitiesForPhysicalGroup` hands them back), element by element.
struct GroupedMsh
{
    int dim = 3;
    std::vector<int64_t> node_tags; ///< file order
    std::vector<std::array<double, 3>> node_coords; ///< file order, always three components
    /// One entry per (group, entity, element) in traversal order: the group's name and tag, the
    /// element's own tag and its vertex tags in the order the file stores them.
    struct Item
    {
        std::string group_name;
        int64_t group_tag = 0;
        int64_t element_tag = 0;
        std::vector<int64_t> nodes;
    };
    std::vector<Item> items;
    /// group tag -> name, in ascending tag order; a group with no name keeps the empty string,
    /// exactly as `gmsh.model.getPhysicalName` returns it.
    std::vector<std::pair<int64_t, std::string>> groups;
};

GroupedMsh read_grouped(const std::string& msh_path)
{
    if (!std::filesystem::exists(msh_path)) {
        log_and_throw_error("File {} does not exist.", msh_path);
    }
    const mshio::MshSpec spec = mshio::load_msh(msh_path);

    GroupedMsh out;
    for (const auto& block : spec.nodes.entity_blocks) {
        for (size_t i = 0; i < block.num_nodes_in_block; ++i) {
            out.node_tags.push_back(static_cast<int64_t>(block.tags[i]));
            out.node_coords.push_back(
                {block.data[3 * i], block.data[3 * i + 1], block.data[3 * i + 2]});
        }
    }

    // Auto-detect the mesh dimension: 3D iff any volume physical group exists, else 2D.
    out.dim = 2;
    for (const auto& ph : spec.physical_groups) {
        if (ph.dim == 3) {
            out.dim = 3;
            break;
        }
    }
    const int elem_type = out.dim == 3 ? 4 : 2; // gmsh element types: 4 = tet, 2 = triangle
    const size_t npp = out.dim == 3 ? 4 : 3;

    std::vector<const mshio::PhysicalGroup*> groups;
    for (const auto& ph : spec.physical_groups) {
        if (ph.dim == out.dim) groups.push_back(&ph);
    }
    std::sort(groups.begin(), groups.end(), [](const auto* a, const auto* b) {
        return a->tag < b->tag;
    });

    std::map<int, std::vector<int>> entity_to_groups;
    if (out.dim == 3) {
        for (const auto& e : spec.entities.volumes) entity_to_groups[e.tag] = e.physical_group_tags;
    } else {
        for (const auto& e : spec.entities.surfaces) {
            entity_to_groups[e.tag] = e.physical_group_tags;
        }
    }

    for (const auto* ph : groups) {
        out.groups.emplace_back(ph->tag, ph->name);
        for (const auto& [ent_tag, group_tags] : entity_to_groups) {
            if (std::find(group_tags.begin(), group_tags.end(), ph->tag) == group_tags.end()) {
                continue;
            }
            for (const auto& block : spec.elements.entity_blocks) {
                if (block.entity_dim != out.dim || block.entity_tag != ent_tag ||
                    block.element_type != elem_type) {
                    continue;
                }
                for (size_t j = 0; j < block.num_elements_in_block; ++j) {
                    const size_t off = j * (npp + 1);
                    GroupedMsh::Item item;
                    item.group_name = ph->name;
                    item.group_tag = ph->tag;
                    item.element_tag = static_cast<int64_t>(block.data[off]);
                    item.nodes.reserve(npp);
                    for (size_t k = 0; k < npp; ++k) {
                        item.nodes.push_back(static_cast<int64_t>(block.data[off + 1 + k]));
                    }
                    out.items.push_back(std::move(item));
                }
            }
        }
    }
    return out;
}

std::vector<int64_t> sorted_copy(const std::vector<int64_t>& v)
{
    std::vector<int64_t> out = v;
    std::sort(out.begin(), out.end());
    return out;
}

/// Python's `str(sorted(vt))` / `str(sorted(non_ambient))` for the refusal message: the message is
/// the only place the two engines can be told apart by eye, so it is spelled the same.
std::string python_list(const std::vector<int64_t>& v)
{
    std::string out = "[";
    for (size_t i = 0; i < v.size(); ++i) {
        if (i != 0) out += ", ";
        out += std::to_string(v[i]);
    }
    return out + "]";
}

std::string python_list(const std::set<std::string>& v)
{
    std::string out = "[";
    bool first = true;
    for (const auto& s : v) {
        if (!first) out += ", ";
        first = false;
        out += "'" + s + "'";
    }
    return out + "]";
}

} // namespace

ReducedBody classify_reduced_cell(
    const TagNames& tags,
    const std::set<std::string>& ambient_like,
    const std::vector<int64_t>& sorted_vertices)
{
    const bool has_ambient = tags.count("ambient") != 0;
    std::set<std::string> non_ambient;
    for (const auto& t : tags) {
        if (ambient_like.count(t) == 0) non_ambient.insert(t);
    }
    if (has_ambient && !non_ambient.empty()) {
        log_and_throw_error(
            "Element with vertices {} has both 'ambient' and non-ambient tags {} — forbidden in "
            "polyfem mesh reduction",
            python_list(sorted_vertices),
            python_list(non_ambient));
    }
    if (!tags.empty() && non_ambient.empty()) {
        return ReducedBody::ambient;
    }
    if (!non_ambient.empty()) {
        return ReducedBody::body;
    }
    return ReducedBody::skip; // tagless cell
}

void write_polyfem_reduced_msh(
    const std::string& input_msh,
    const std::string& output_msh,
    const std::vector<std::string>& ambient_like_tags)
{
    const GroupedMsh in = read_grouped(input_msh);
    const int prim_dim = in.dim;
    const int elem_type = prim_dim == 3 ? 4 : 2;

    // Sorted by tag so the reduced mesh's storage order is monotonic; see the header.
    std::vector<size_t> order(in.node_tags.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&in](size_t a, size_t b) {
        return in.node_tags[a] < in.node_tags[b];
    });

    // Cells in first-appearance order, keyed by their vertex SET; the first copy's vertex ORDER is
    // the one written, and every copy's group name joins the cell's tag set.
    std::map<std::vector<int64_t>, size_t> index_of;
    std::vector<std::vector<int64_t>> cell_nodes;
    std::vector<TagNames> cell_tags;
    for (const auto& item : in.items) {
        const std::vector<int64_t> key = sorted_copy(item.nodes);
        auto [it, inserted] = index_of.emplace(key, cell_nodes.size());
        if (inserted) {
            cell_nodes.push_back(item.nodes);
            cell_tags.emplace_back();
        }
        cell_tags[it->second].insert(item.group_name);
    }

    std::set<std::string> ambient_like(ambient_like_tags.begin(), ambient_like_tags.end());
    ambient_like.insert("ambient");

    std::vector<std::vector<int64_t>> ambient_prims;
    std::vector<std::vector<int64_t>> body_prims;
    for (size_t c = 0; c < cell_nodes.size(); ++c) {
        switch (classify_reduced_cell(cell_tags[c], ambient_like, sorted_copy(cell_nodes[c]))) {
        case ReducedBody::ambient: ambient_prims.push_back(cell_nodes[c]); break;
        case ReducedBody::body: body_prims.push_back(cell_nodes[c]); break;
        case ReducedBody::skip: break;
        }
    }

    // Two discrete entities, tags 1 and 2, one per physical group; the nodes all live on the
    // ambient entity, as they do in the Python (entity ownership does not affect the element
    // vertex references, which are node tags).
    mshio::MshSpec spec;
    spec.mesh_format.version = "4.1";
    spec.mesh_format.file_type = 1; // binary: see the header
    spec.mesh_format.data_size = sizeof(size_t);
    spec.physical_groups.push_back({prim_dim, 1, "ambient"});
    spec.physical_groups.push_back({prim_dim, 2, "body"});
    if (prim_dim == 3) {
        spec.entities.volumes.push_back({1, 0, 0, 0, 0, 0, 0, {1}, {}});
        spec.entities.volumes.push_back({2, 0, 0, 0, 0, 0, 0, {2}, {}});
    } else {
        spec.entities.surfaces.push_back({1, 0, 0, 0, 0, 0, 0, {1}, {}});
        spec.entities.surfaces.push_back({2, 0, 0, 0, 0, 0, 0, {2}, {}});
    }

    mshio::NodeBlock nodes;
    nodes.entity_dim = prim_dim;
    nodes.entity_tag = 1;
    nodes.num_nodes_in_block = order.size();
    nodes.tags.reserve(order.size());
    nodes.data.reserve(3 * order.size());
    for (const size_t i : order) {
        nodes.tags.push_back(static_cast<size_t>(in.node_tags[i]));
        nodes.data.push_back(in.node_coords[i][0]);
        nodes.data.push_back(in.node_coords[i][1]);
        nodes.data.push_back(in.node_coords[i][2]);
    }
    spec.nodes.num_entity_blocks = 1;
    spec.nodes.num_nodes = order.size();
    spec.nodes.min_node_tag = nodes.tags.empty() ? 0 : nodes.tags.front();
    spec.nodes.max_node_tag = nodes.tags.empty() ? 0 : nodes.tags.back();
    spec.nodes.entity_blocks.push_back(std::move(nodes));

    size_t next_id = 1;
    const auto add_block = [&](int entity_tag, const std::vector<std::vector<int64_t>>& prims) {
        if (prims.empty()) return;
        mshio::ElementBlock block;
        block.entity_dim = prim_dim;
        block.entity_tag = entity_tag;
        block.element_type = elem_type;
        block.num_elements_in_block = prims.size();
        for (const auto& prim : prims) {
            block.data.push_back(next_id++);
            for (const int64_t v : prim) block.data.push_back(static_cast<size_t>(v));
        }
        spec.elements.entity_blocks.push_back(std::move(block));
    };
    add_block(1, ambient_prims);
    add_block(2, body_prims);
    spec.elements.num_entity_blocks = spec.elements.entity_blocks.size();
    spec.elements.num_elements = ambient_prims.size() + body_prims.size();
    spec.elements.min_element_tag = 1;
    spec.elements.max_element_tag = next_id - 1;

    mshio::save_msh(output_msh, spec);
    logger().info(
        "  reduced  : {}  ({} ambient + {} body {})",
        output_msh,
        ambient_prims.size(),
        body_prims.size(),
        prim_dim == 3 ? "tets" : "triangles");
}

MeshInfo get_mesh_info(const std::string& msh_path)
{
    const GroupedMsh in = read_grouped(msh_path);

    MeshInfo info;
    info.dim = in.dim;
    std::map<int64_t, size_t> row_of;
    for (size_t i = 0; i < in.node_tags.size(); ++i) row_of[in.node_tags[i]] = i;

    std::set<int64_t> tags;
    for (const auto& [tag, name] : in.groups) {
        if (!name.empty()) info.name_to_tag[name] = tag;
        tags.insert(tag);

        // Elements of the group, deduped by element tag and kept in traversal order: a group that
        // spans several entities may list an element twice, and the Python's `seen` dict counts
        // and integrates it once, at the position of its first appearance.
        std::vector<const std::vector<int64_t>*> seen;
        std::set<int64_t> seen_tags;
        for (const auto& item : in.items) {
            if (item.group_tag != tag) continue;
            if (seen_tags.insert(item.element_tag).second) seen.push_back(&item.nodes);
        }

        // Running (not pairwise) sum, in this order; see the header.
        double vol = 0.0;
        for (const auto* nodes : seen) {
            std::array<std::array<double, 3>, 4> v{};
            for (size_t k = 0; k < nodes->size(); ++k) {
                v[k] = in.node_coords[row_of.at((*nodes)[k])];
            }
            if (in.dim == 3) {
                double edges[3][3];
                for (int e = 0; e < 3; ++e) {
                    for (int d = 0; d < 3; ++d) edges[e][d] = v[e + 1][d] - v[0][d];
                }
                vol += std::fabs(numpy_det3(edges)) / 6.0;
            } else {
                const double e1x = v[1][0] - v[0][0];
                const double e1y = v[1][1] - v[0][1];
                const double e2x = v[2][0] - v[0][0];
                const double e2y = v[2][1] - v[0][1];
                // Two rounded products, then a subtraction, then the halving: the Python is
                // `0.5 * abs(e1[0] * e2[1] - e1[1] * e2[0])` on numpy scalars, which never fuses.
                const double p1 = e1x * e2y;
                const double p2 = e1y * e2x;
                vol += 0.5 * std::fabs(p1 - p2);
            }
        }
        info.tag_to_count[tag] = static_cast<int64_t>(seen.size());
        info.tag_to_volume[tag] = vol;
    }
    info.tags.assign(tags.begin(), tags.end());
    return info;
}

} // namespace wmtk::components::polyfem_ops
