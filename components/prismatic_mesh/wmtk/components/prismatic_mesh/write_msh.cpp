#include "write_msh.hpp"

#include <mshio/mshio.h>
#include <array>
#include <fstream>
#include <iomanip>
#include <limits>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh::detail {
namespace {
int gmsh_type(paraviewo::CellType type)
{
    if (type == paraviewo::CellType::Tetrahedron) return 4;
    if (type == paraviewo::CellType::Wedge) return 6;
    if (type == paraviewo::CellType::Pyramid) return 7;
    log_and_throw_error("Unsupported cell type in prismatic MSH export.");
    return -1;
}

void add_data(
    std::vector<mshio::Data>& destination,
    const MeshFields& fields,
    const std::vector<size_t>& output_rows)
{
    for (const auto& [name, values] : fields) {
        if (values.rows() != static_cast<Eigen::Index>(output_rows.size()) ||
            values.rows() > std::numeric_limits<int>::max())
            log_and_throw_error("Invalid MSH field size: {}", name);
        // Gmsh views accept only 1, 3 or 9 components. Preserve the 12-component
        // reference decomposition as individually named scalar fields.
        const bool split = values.cols() != 1 && values.cols() != 3 && values.cols() != 9;
        const int components = split ? 1 : static_cast<int>(values.cols());
        const int parts = split ? static_cast<int>(values.cols()) : 1;
        for (int part = 0; part < parts; ++part) {
            mshio::Data data;
            data.header.string_tags = {split ? name + "_" + std::to_string(part) : name};
            data.header.real_tags = {0};
            data.header.int_tags = {0, components, static_cast<int>(output_rows.size())};
            data.entries.reserve(output_rows.size());
            for (size_t i = 0; i < output_rows.size(); ++i) {
                mshio::DataEntry entry;
                entry.tag = i + 1;
                for (int j = 0; j < components; ++j)
                    entry.data.push_back(values(output_rows[i], split ? part : j));
                data.entries.push_back(std::move(entry));
            }
            destination.push_back(std::move(data));
        }
    }
}
} // namespace

void write_msh(
    const std::filesystem::path& path,
    const MatrixXd& vertices,
    const std::vector<paraviewo::CellElement>& cells,
    const MeshFields& point_fields,
    const MeshFields& cell_fields)
{
    mshio::MshSpec spec;
    spec.mesh_format.version = "4.1";
    spec.mesh_format.file_type = 0;
    spec.mesh_format.data_size = sizeof(double);

    // Group volume elements by region and Gmsh type. Assign consecutive element
    // tags in file order, then apply that same permutation to every ElementData field.
    std::map<std::pair<int, int>, std::vector<size_t>> groups;
    std::vector<int> node_region(vertices.rows(), 0);
    std::array<std::vector<size_t>, 4> region_vertices;
    for (size_t i = 0; i < cells.size(); ++i) {
        const int region = cell_fields.at("offset_tag")(i, 0) == 1 ? 2
                           : cell_fields.at("tag_0")(i, 0) == 1    ? 1
                                                                   : 3;
        const int type = gmsh_type(cells[i].ctype);
        if (cells[i].vertices.size() != mshio::nodes_per_element(type))
            log_and_throw_error("Invalid corner count in MSH export.");
        groups[{region, type}].push_back(i);
        for (int v : cells[i].vertices) {
            if (v < 0 || v >= vertices.rows())
                log_and_throw_error("Invalid vertex index in MSH export.");
            if (node_region[v] == 0) node_region[v] = region;
            region_vertices[region].push_back(v);
        }
    }

    std::map<int, mshio::NodeBlock> node_blocks;
    std::vector<size_t> vertex_rows;
    std::vector<size_t> node_tags(vertices.rows());
    for (size_t v = 0; v < node_region.size(); ++v) {
        // Retained isolated input points are still exported, even with no input cells.
        if (node_region[v] == 0) {
            node_region[v] = 1;
            region_vertices[1].push_back(v);
        }
        auto& block = node_blocks[node_region[v]];
        block.entity_dim = 3;
        block.entity_tag = node_region[v];
        block.tags.push_back(v + 1);
        for (int j = 0; j < 3; ++j) block.data.push_back(vertices(v, j));
        ++block.num_nodes_in_block;
    }
    for (auto& [region, block] : node_blocks) {
        // Keep node tags consecutive in file order too, with the same permutation
        // applied to NodeData and every element's connectivity.
        for (size_t& tag : block.tags) {
            const size_t row = tag - 1;
            vertex_rows.push_back(row);
            tag = vertex_rows.size();
            node_tags[row] = tag;
        }
        spec.nodes.entity_blocks.push_back(std::move(block));
    }
    spec.nodes.num_entity_blocks = spec.nodes.entity_blocks.size();
    spec.nodes.num_nodes = vertices.rows();
    spec.nodes.min_node_tag = vertices.rows() ? 1 : 0;
    spec.nodes.max_node_tag = vertices.rows();

    const std::array<std::string, 4> region_names = {"", "input", "offset_band", "background"};
    for (int region = 1; region <= 3; ++region) {
        if (region_vertices[region].empty()) continue;
        Vector3d lo = vertices.row(region_vertices[region][0]).transpose(), hi = lo;
        for (size_t v : region_vertices[region]) {
            lo = lo.cwiseMin(vertices.row(v).transpose());
            hi = hi.cwiseMax(vertices.row(v).transpose());
        }
        mshio::VolumeEntity entity;
        entity.tag = region;
        entity.min_x = lo[0];
        entity.min_y = lo[1];
        entity.min_z = lo[2];
        entity.max_x = hi[0];
        entity.max_y = hi[1];
        entity.max_z = hi[2];
        entity.physical_group_tags = {region};
        spec.entities.volumes.push_back(entity);
        spec.physical_groups.push_back({3, region, region_names[region]});
    }

    std::vector<size_t> cell_rows;
    for (const auto& [key, rows] : groups) {
        mshio::ElementBlock block;
        block.entity_dim = 3;
        block.entity_tag = key.first;
        block.element_type = key.second;
        block.num_elements_in_block = rows.size();
        for (size_t row : rows) {
            block.data.push_back(cell_rows.size() + 1);
            // Linear tet/prism/pyramid corners use the same order as our VTK cells.
            // Only the element type and the one-based global node tags change.
            for (int v : cells[row].vertices) block.data.push_back(node_tags[v]);
            cell_rows.push_back(row);
        }
        spec.elements.entity_blocks.push_back(std::move(block));
    }
    spec.elements.num_entity_blocks = spec.elements.entity_blocks.size();
    spec.elements.num_elements = cells.size();
    spec.elements.min_element_tag = cells.empty() ? 0 : 1;
    spec.elements.max_element_tag = cells.size();
    add_data(spec.node_data, point_fields, vertex_rows);
    add_data(spec.element_data, cell_fields, cell_rows);
    mshio::validate_spec(spec);

    std::ofstream out(path);
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    mshio::save_msh(out, spec);
    out.close();
    if (!out) log_and_throw_error("Could not write result mesh: {}", path.string());
}

} // namespace wmtk::components::prismatic_mesh::detail
