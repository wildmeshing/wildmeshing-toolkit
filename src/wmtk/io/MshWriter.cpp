#include "MshWriter.hpp"

#include <numeric>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::io {

MshWriter::MshWriter(const std::filesystem::path& filename)
    : m_name(filename)
{}

void MshWriter::write(const Mesh& mesh, const std::string& position_attribute_name)
{
    auto pos_handle =
        mesh.get_attribute_handle<double>(position_attribute_name, PrimitiveType::Vertex);
    auto pos_acc = mesh.create_const_accessor<double>(pos_handle);

    const auto vertices = mesh.get_all_id_simplex(PrimitiveType::Vertex);
    const auto cells = mesh.get_all_id_simplex(mesh.top_simplex_type());

    const int64_t dim = (int64_t)mesh.top_simplex_type();
    const int64_t pos_dim = pos_handle.dimension();

    auto& format = m_spec.mesh_format;
    format.version = "4.1"; // Only version "2.2" and "4.1" are supported.
    format.file_type = 1; // 0: ASCII, 1: binary.
    format.data_size = sizeof(size_t); // Size of data, defined as sizeof(size_t) = 8.

    // nodes
    {
        auto& nodes = m_spec.nodes;
        nodes.num_entity_blocks = 1; // Number of node blocks.
        nodes.num_nodes = vertices.size(); // Total number of nodes.
        nodes.min_node_tag = 1;
        nodes.max_node_tag = vertices.size();
        //nodes.entity_blocks = {...}; // A std::vector of node blocks.
        nodes.entity_blocks.push_back({});

        auto& block = nodes.entity_blocks[0];
        block.entity_dim = pos_dim; // The dimension of the entity.
        block.entity_tag = 1; // The entity these nodes belongs to.
        block.parametric = 0; // 0: non-parametric, 1: parametric.
        block.num_nodes_in_block = vertices.size(); // The number of nodes in block.
        //block.tags = {...}; // A std::vector of unique, positive node tags.
        block.tags.resize(vertices.size());
        std::iota(block.tags.begin(), block.tags.end(), 1);
        //block.data = {...}; // A std::vector of coordinates (x,y,z,<u>,<v>,<w>,...)

        block.data.resize(vertices.size() * pos_dim);
        for (const auto& v : vertices) {
            const int64_t idx = mesh.id(v);
            const auto p = pos_acc.const_vector_attribute(v);
            for (int64_t i = 0; i < pos_dim; ++i) {
                block.data[idx * pos_dim + i] = p(i);
            }
        }
    }

    // elements
    {
        auto& elements = m_spec.elements;
        elements.num_entity_blocks = 1; // Number of element blocks.
        elements.num_elements = cells.size(); // Total number of elmeents.
        elements.min_element_tag = 1;
        elements.max_element_tag = cells.size();
        //elements.entity_blocks = {...}; // A std::vector of element blocks.
        elements.entity_blocks.push_back({});

        auto& block = elements.entity_blocks[0];
        block.entity_dim = dim; // The dimension of the elements.
        block.entity_tag = 1; // The entity these elements belongs to.
        switch (dim) {
        case 1: // edge
            block.element_type = 1; // See element type table of mshio.
            break;
        case 2: // triangle
            block.element_type = 2; // See element type table of mshio.
            break;
        case 3: // tet
            block.element_type = 4; // See element type table of mshio.
            break;
        default: log_and_throw_error("Element type not supported for writing to MSH.");
        }
        block.num_elements_in_block = cells.size(); // The number of elements in this block.
        //block.data = {...}; // See more detail below.

        block.data.resize(cells.size() * (dim + 2));
        for (const auto& c : cells) {
            const int64_t idx = mesh.id(c);
            const auto vs = mesh.orient_vertices(mesh.get_tuple_from_id_simplex(c));
            assert(vs.size() == dim + 1);

            // block data holds entries for every cell: element_tag node_tag ... node_tag
            block.data[idx * (dim + 2) + 0] = idx + 1;
            for (int64_t i = 0; i < vs.size(); ++i) {
                const int64_t vid = mesh.id(vs[i], PrimitiveType::Vertex);
                block.data[idx * (dim + 2) + 1 + i] = vid + 1;
            }
        }
    }

    mshio::save_msh(m_name.string(), m_spec);
}

} // namespace wmtk::io