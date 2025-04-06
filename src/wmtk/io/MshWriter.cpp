#include "MshWriter.hpp"

#include <numeric>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::io {

void MshWriter::write(
    const std::filesystem::path& m_name,
    const Mesh& mesh,
    const std::string& position_attribute_name,
    const std::vector<std::string>& cell_attribute_names)
{
    auto pos_handle =
        mesh.get_attribute_handle<double>(position_attribute_name, PrimitiveType::Vertex);
    auto pos_acc = mesh.create_const_accessor<double>(pos_handle);

    const auto vertices = mesh.get_all_id_simplex(PrimitiveType::Vertex);
    const auto cells = mesh.get_all_id_simplex(mesh.top_simplex_type());

    const int64_t dim = (int64_t)mesh.top_simplex_type();
    const int64_t pos_dim = pos_handle.dimension();

    mshio::MshSpec spec;

    auto& format = spec.mesh_format;
    format.version = "4.1"; // Only version "2.2" and "4.1" are supported.
    format.file_type = 1; // 0: ASCII, 1: binary.
    format.data_size = sizeof(size_t); // Size of data, defined as sizeof(size_t) = 8.

    // nodes
    {
        auto& nodes = spec.nodes;
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
        auto& elements = spec.elements;
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

    // element attributes
    for (const std::string& attribute_name : cell_attribute_names) {
        const auto attr =
            mesh.get_attribute_handle_typed<double>(attribute_name, mesh.top_simplex_type());
        const auto acc = mesh.create_const_accessor(attr);

        spec.element_data.push_back({});
        auto& data = spec.element_data.back();

        auto& header = data.header;
        header.string_tags.push_back(attribute_name); // [field_name, <interpolation_scheme>, ...]
        header.real_tags.push_back(0); // [<time value>, ...]
        header.int_tags.resize(3); // [time step, num fields, num entities, <partition id>, ...]
        header.int_tags[0] = 0;
        header.int_tags[1] = acc.dimension();
        header.int_tags[2] = cells.size();

        data.entries.resize(cells.size());
        for (const auto& c : cells) {
            const int64_t idx = mesh.id(c);
            auto& entry = data.entries[idx];

            entry.tag = idx + 1;
            // entry.num_nodes_per_element = dim + 1;
            entry.data.resize(acc.dimension());

            auto a = acc.const_vector_attribute(c);
            for (int64_t i = 0; i < acc.dimension(); ++i) {
                entry.data[i] = a(i);
            }
        }
    }

    mshio::save_msh(m_name.string(), spec);
}

} // namespace wmtk::io