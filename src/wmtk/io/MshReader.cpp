#include "MshReader.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk {


std::shared_ptr<Mesh> MshReader::read(const std::filesystem::path& filename, bool ignore_z)
{
    m_spec = mshio::load_msh(filename.string());
    m_ignore_z = ignore_z;

    std::shared_ptr<Mesh> res;

    if (get_num_tets() > 0) {
        assert(!m_ignore_z);
        V.resize(get_num_tet_vertices(), 3);
        S.resize(get_num_tets(), 4);

        extract_tet_vertices();
        extract_tets();

        auto tmp = std::make_shared<TetMesh>();
        tmp->initialize(S);
        res = tmp;
    } else if (get_num_faces() > 0) {
        V.resize(get_num_face_vertices(), m_ignore_z ? 2 : 3);
        S.resize(get_num_faces(), 3);

        extract_face_vertices();
        extract_faces();

        auto tmp = std::make_shared<TriMesh>();
        tmp->initialize(S);
        res = tmp;
    } else if (get_num_edges() > 0) {
        V.resize(get_num_face_vertices(), m_ignore_z ? 2 : 3);
        S.resize(get_num_edges(), 2);

        extract_edge_vertices();
        extract_edges();

        auto tmp = std::make_shared<EdgeMesh>();
        tmp->initialize(S);
        res = tmp;
    } else {
        res = std::make_shared<PointMesh>();
    }


    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *res);

    return res;
}


void MshReader::extract_tet_vertices()
{
    return extract_vertices<3>();
}

void MshReader::extract_face_vertices()
{
    return extract_vertices<2>();
}

void MshReader::extract_edge_vertices()
{
    extract_vertices<1>();
}

void MshReader::extract_edges()
{
    extract_simplex_elements<1>();
}

void MshReader::extract_faces()
{
    extract_simplex_elements<2>();
}

void MshReader::extract_tets()
{
    extract_simplex_elements<3>();
}

// std::vector<std::string> MshReader::get_edge_vertex_attribute_names() const
// {
//     return get_vertex_attribute_names<1>();
// }

// std::vector<std::string> MshReader::get_face_vertex_attribute_names() const
// {
//     return get_vertex_attribute_names<2>();
// }

// std::vector<std::string> MshReader::get_tet_vertex_attribute_names() const
// {
//     return get_vertex_attribute_names<3>();
// }

// std::vector<std::string> MshReader::get_edge_attribute_names() const
// {
//     return get_element_attribute_names<1>();
// }

// std::vector<std::string> MshReader::get_face_attribute_names() const
// {
//     return get_element_attribute_names<2>();
// }

// std::vector<std::string> MshReader::get_tet_attribute_names() const
// {
//     return get_element_attribute_names<3>();
// }

// void MshReader::extract_edge_vertex_attribute(const std::string& attr_name)
// {
//     extract_vertex_attribute<1>(attr_name, std::forward<Fn>(set_attr));
// }

// void MshReader::extract_face_vertex_attribute(const std::string& attr_name)
// {
//     extract_vertex_attribute<2>(attr_name, std::forward<Fn>(set_attr));
// }

// void MshReader::extract_tet_vertex_attribute(const std::string& attr_name)
// {
//     extract_vertex_attribute<3>(attr_name, std::forward<Fn>(set_attr));
// }

// void MshReader::extract_edge_attribute(const std::string& attr_name)
// {
//     extract_element_attribute<1>(attr_name, std::forward<Fn>(set_attr));
// }

// void MshReader::extract_face_attribute(const std::string& attr_name)
// {
//     extract_element_attribute<2>(attr_name, std::forward<Fn>(set_attr));
// }

// void MshReader::extract_tet_attribute(const std::string& attr_name)
// {
//     extract_element_attribute<3>(attr_name, std::forward<Fn>(set_attr));
// }


template <int DIM>
const mshio::NodeBlock* MshReader::get_vertex_block() const
{
    for (const auto& block : m_spec.nodes.entity_blocks) {
        if (block.entity_dim == DIM) {
            return &block;
        }
    }
    return nullptr;
}

template <int DIM>
const mshio::ElementBlock* MshReader::get_simplex_element_block() const
{
    for (const auto& block : m_spec.elements.entity_blocks) {
        if (block.entity_dim == DIM) {
            return &block;
        }
    }
    return nullptr;
}

template <int DIM>
size_t MshReader::get_num_vertices() const
{
    const auto* block = get_vertex_block<DIM>();
    if (block != nullptr) {
        return block->num_nodes_in_block;
    } else {
        return 0;
    }
}

template <int DIM>
size_t MshReader::get_num_simplex_elements() const
{
    const auto* block = get_simplex_element_block<DIM>();
    if (block != nullptr) {
        return block->num_elements_in_block;
    } else {
        return 0;
    }
}

template <int DIM>
void MshReader::extract_vertices()
{
    const auto* block = get_vertex_block<DIM>();
    if (block == nullptr) return;

    const size_t num_vertices = block->num_nodes_in_block;
    if (num_vertices == 0) return;

    const size_t tag_offset = block->tags.front();
    for (size_t i = 0; i < num_vertices; i++) {
        size_t tag = block->tags[i] - tag_offset;
        set_vertex(tag, block->data[i * 3], block->data[i * 3 + 1], block->data[i * 3 + 2]);
    }
}

template <int DIM>
void MshReader::extract_simplex_elements()
{
    const auto* vertex_block = get_vertex_block<DIM>();
    const auto* element_block = get_simplex_element_block<DIM>();
    if (element_block == nullptr) return;
    assert(vertex_block != nullptr);

    const size_t num_elements = element_block->num_elements_in_block;
    if (num_elements == 0) return;
    assert(vertex_block->num_nodes_in_block != 0);

    const size_t vert_tag_offset = vertex_block->tags.front();
    const size_t elem_tag_offset = element_block->data.front();
    for (size_t i = 0; i < num_elements; i++) {
        const size_t tag = element_block->data[i * (DIM + 2)] - elem_tag_offset;
        assert(tag < num_elements);
        const auto* element = element_block->data.data() + i * (DIM + 2) + 1;

        if constexpr (DIM == 1) {
            set_edge(tag, element[0] - vert_tag_offset, element[1] - vert_tag_offset);
        } else if constexpr (DIM == 2) {
            set_face(
                tag,
                element[0] - vert_tag_offset,
                element[1] - vert_tag_offset,
                element[2] - vert_tag_offset);
        } else if constexpr (DIM == 3) {
            set_tet(
                tag,
                element[0] - vert_tag_offset,
                element[1] - vert_tag_offset,
                element[2] - vert_tag_offset,
                element[3] - vert_tag_offset);
        }
    }
}

// Old code to read attribute, we might need it in the future
// template <int DIM>
// std::vector<std::string> get_vertex_attribute_names() const
// {
//     std::vector<std::string> attr_names;
//     attr_names.reserve(m_spec.node_data.size());
//     for (const auto& data : m_spec.node_data) {
//         const auto& int_tags = data.header.int_tags;
//         if (int_tags.size() >= 5 && int_tags[4] == DIM) {
//             attr_names.push_back(data.header.string_tags.front());
//         }
//     }
//     return attr_names;
// }

// template <int DIM>
// std::vector<std::string> get_element_attribute_names() const
// {
//     std::vector<std::string> attr_names;
//     attr_names.reserve(m_spec.element_data.size());
//     for (const auto& data : m_spec.element_data) {
//         const auto& int_tags = data.header.int_tags;
//         if (int_tags.size() >= 5 && int_tags[4] == DIM) {
//             attr_names.push_back(data.header.string_tags.front());
//         }
//     }
//     return attr_names;
// }

// template <int DIM, typename Fn>
// void extract_vertex_attribute(const std::string& attr_name, Fn&& set_attr)
// {
//     const auto* vertex_block = get_vertex_block<DIM>();
//     const size_t tag_offset = vertex_block->tags.front();
//     for (const auto& data : m_spec.node_data) {
//         if (data.header.string_tags.front() != attr_name) continue;
//         if (data.header.int_tags.size() >= 5 && data.header.int_tags[4] != DIM) {
//             throw std::runtime_error("Attribute " + attr_name + " is of the wrong DIM.");
//         }

//         for (const auto& entry : data.entries) {
//             const size_t tag = entry.tag - tag_offset;
//             assert(tag < vertex_block->num_nodes_in_block);
//             set_attr(tag, entry.data);
//         }
//     }
// }

// template <int DIM, typename Fn>
// void extract_element_attribute(const std::string& attr_name, Fn&& set_attr)
// {
//     const auto* element_block = get_simplex_element_block<DIM>();
//     const size_t tag_offset = element_block->data.front();
//     for (const auto& data : m_spec.element_data) {
//         if (data.header.string_tags.front() != attr_name) continue;
//         if (data.header.int_tags.size() >= 5 && data.header.int_tags[4] != DIM) {
//             throw std::runtime_error("Attribute " + attr_name + " is of the wrong DIM.");
//         }

//         for (const auto& entry : data.entries) {
//             const size_t tag = entry.tag - tag_offset;
//             assert(tag < element_block->num_elements_in_block);
//             set_attr(tag, entry.data);
//         }
//     }
// }


} // namespace wmtk