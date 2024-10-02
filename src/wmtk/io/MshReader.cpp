#include "MshReader.hpp"

#include <set>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk/utils/orient.hpp>

#include "predicates.h"

namespace wmtk {

// reading from an msh file while preserve the specified attributes, only supports valid scalar
// attributes on tets in a tetmesh
std::shared_ptr<Mesh> MshReader::read(
    const std::filesystem::path& filename,
    bool ignore_z,
    const std::vector<std::string>& tetrahedron_attributes)
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

        exactinit();

        {
            // check inversion
            Eigen::Vector3d p0 = V.row(S(0, 0));
            Eigen::Vector3d p1 = V.row(S(0, 1));
            Eigen::Vector3d p2 = V.row(S(0, 2));
            Eigen::Vector3d p3 = V.row(S(0, 3));

            if (wmtk::utils::wmtk_orient3d(p0, p1, p2, p3) < 0) {
                // swap col 0 and 1 of S
                S.col(0).swap(S.col(1));
                wmtk::logger().info(
                    "Input tet orientation is inverted, swapping col 0 and 1 of TV matirx.");
            }
        }

        {
            // check consistency
            for (int64_t i = 0; i < S.rows(); i++) {
                Eigen::Vector3d p0 = V.row(S(i, 0));
                Eigen::Vector3d p1 = V.row(S(i, 1));
                Eigen::Vector3d p2 = V.row(S(i, 2));
                Eigen::Vector3d p3 = V.row(S(i, 3));
                auto orient = wmtk::utils::wmtk_orient3d(p0, p1, p2, p3);

                if (orient < 0) {
                    auto tmp = S(i, 1);
                    S(i, 1) = S(i, 0);
                    S(i, 0) = tmp;
                    // log_and_throw_error("Input tet orientation is inconsistent.");
                } else if (orient == 0) {
                    log_and_throw_error("Input tet is degenerated.");
                }
            }
        }

        {
            // check consistency
            for (int64_t i = 0; i < S.rows(); i++) {
                Eigen::Vector3d p0 = V.row(S(i, 0));
                Eigen::Vector3d p1 = V.row(S(i, 1));
                Eigen::Vector3d p2 = V.row(S(i, 2));
                Eigen::Vector3d p3 = V.row(S(i, 3));
                auto orient = wmtk::utils::wmtk_orient3d(p0, p1, p2, p3);

                if (orient < 0) {
                    V.row(S(i, 0)) = p1;
                    V.row(S(i, 1)) = p0;
                    // log_and_throw_error("Input tet orientation is inconsistent.");
                } else if (orient == 0) {
                    log_and_throw_error("Input tet is degenerated.");
                }
            }
        }

        auto tmp = std::make_shared<TetMesh>();
        tmp->initialize(S);
        {
            // save all valid scalar attributes on tets
            std::set<std::string> valid_attr_names;
            for (const auto& data : m_spec.element_data) {
                // validness: int_tags[2] stores the dimension of an element_data, which should
                // match the size of tets
                if (data.header.int_tags[2] != get_num_tets()) {
                    continue;
                }
                valid_attr_names.insert(data.header.string_tags.front());
            }
            for (const std::string& attr : tetrahedron_attributes) {
                if (valid_attr_names.count(attr) == 0) {
                    log_and_throw_error("Attribute " + attr + " does not exist in the msh file.");
                }
                extract_element_attribute(tmp, attr, 3);
            }
        }
        res = tmp;
    } else if (get_num_faces() > 0) {
        assert(tetrahedron_attributes.empty());

        V.resize(get_num_face_vertices(), m_ignore_z ? 2 : 3);
        S.resize(get_num_faces(), 3);

        extract_face_vertices();
        extract_faces();

        exactinit();

        auto tmp = std::make_shared<TriMesh>();
        tmp->initialize(S);
        res = tmp;

    } else if (get_num_edges() > 0) {
        assert(tetrahedron_attributes.empty());

        V.resize(get_num_edge_vertices(), m_ignore_z ? 2 : 3);
        S.resize(get_num_edges(), 2);

        extract_edge_vertices();
        extract_edges();

        auto tmp = std::make_shared<EdgeMesh>();
        tmp->initialize(S);
        res = tmp;
    } else {
        assert(tetrahedron_attributes.empty());

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
// template <int DIM, typename Fn>
// void MshReader::extract_edge_vertex_attribute(const std::string& attr_name)
// {
//     extract_vertex_attribute<1>(attr_name, std::forward<Fn>(set_attr));
// }
// template <int DIM, typename Fn>
// void MshReader::extract_face_vertex_attribute(const std::string& attr_name)
// {
//     extract_vertex_attribute<2>(attr_name, std::forward<Fn>(set_attr));
// }
// template <int DIM, typename Fn>
// void MshReader::extract_tet_vertex_attribute(const std::string& attr_name)
// {
//     extract_vertex_attribute<3>(attr_name, std::forward<Fn>(set_attr));
// }
// template <int DIM, typename Fn>
// void MshReader::extract_edge_attribute(const std::string& attr_name)
// {
//     extract_element_attribute<1>(attr_name, std::forward<Fn>(set_attr));
// }
// template <int DIM, typename Fn>
// void MshReader::extract_face_attribute(const std::string& attr_name)
// {
//     extract_element_attribute<2>(attr_name, std::forward<Fn>(set_attr));
// }
// template <int DIM, typename Fn>
// void MshReader::extract_tet_attribute(const std::string& attr_name)
// {
//     extract_element_attribute<3>(attr_name, std::forward<Fn>(set_attr));
// }


const mshio::NodeBlock* MshReader::get_vertex_block(int DIM) const
{
    for (const auto& block : m_spec.nodes.entity_blocks) {
        if (block.entity_dim == DIM) {
            return &block;
        }
    }
    return nullptr;
}

const mshio::ElementBlock* MshReader::get_simplex_element_block(int DIM) const
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
    const auto* block = get_vertex_block(DIM);
    if (block != nullptr) {
        return block->num_nodes_in_block;
    } else {
        return 0;
    }
}

template <int DIM>
size_t MshReader::get_num_simplex_elements() const
{
    const auto* block = get_simplex_element_block(DIM);
    if (block != nullptr) {
        return block->num_elements_in_block;
    } else {
        return 0;
    }
}

template <int DIM>
void MshReader::extract_vertices()
{
    const auto* block = get_vertex_block(DIM);
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
    const auto* vertex_block = get_vertex_block(DIM);
    const auto* element_block = get_simplex_element_block(DIM);
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
// std::vector<std::string> MshReader::get_vertex_attribute_names(int DIM) const
//{
//    std::vector<std::string> attr_names;
//    attr_names.reserve(m_spec.node_data.size());
//    for (const auto& data : m_spec.node_data) {
//        const auto& int_tags = data.header.int_tags;
//        if (int_tags.size() >= 5 && int_tags[4] == DIM) {
//            attr_names.push_back(data.header.string_tags.front());
//        }
//    }
//    return attr_names;
//}

// std::vector<std::string> MshReader::get_element_attribute_names(int DIM) const
//{
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

// void MshReader::extract_vertex_attribute(
//     std::shared_ptr<wmtk::TetMesh> m,
//     const std::string& attr_name,
//     int DIM)
//{
//     const auto* vertex_block = get_vertex_block(DIM);
//     const size_t tag_offset = vertex_block->tags.front();
//     for (const auto& data : m_spec.node_data) {
//         if (data.header.string_tags.front() != attr_name) continue;
//         if (data.header.int_tags.size() >= 5 && data.header.int_tags[4] != DIM) {
//             throw std::runtime_error("Attribute " + attr_name + " is of the wrong DIM.");
//         }
//
//        // for (const auto& entry : data.entries) {
//        //     const size_t tag = entry.tag - tag_offset;
//        //     assert(tag < vertex_block->num_nodes_in_block);
//        //     set_attr(tag, entry.data);
//        // }
//
//        // figure out what data type the attribute stores
//        // ...
//
//        for (const auto& entry : data.entries) {
//            // auto attr_handle = m->register_attribute<double>(
//            //     std::string(attr_name) + std::to_string(index),
//            //     PrimitiveType::Tetrahedron,
//            //     m->get_all(m->top_simplex_type()).size());
//            // auto acc = m->create_accessor<double>(attr_handle);
//            const size_t tag = entry.tag - tag_offset;
//            assert(tag < vertex_block->num_nodes_in_block);
//            // set_attr(tag, entry.data);
//
//            Eigen::VectorX<long> eigendata;
//            eigendata.resize(entry.data.size());
//            for (int i = 0; i < entry.data.size(); ++i) {
//                eigendata.row(i) << entry.data[i];
//            }
//            // wmtk::attribute::MeshAttributeHandle tag_handle =
//            //     wmtk::mesh_utils::set_matrix_attribute(
//            //         eigendata,
//            //         std::string(attr_name) + std::to_string(index),
//            //         PrimitiveType::Tetrahedron,
//            //         *(m.get()));
//            // for (const wmtk::Tuple& tup : m->get_all(m->top_simplex_type())) {
//            //     acc.scalar_attribute(tup) = entry.data;
//            // }
//        }
//    }
//}

// for now, this function only supports reading a single attribute on the tet mesh
void MshReader::extract_element_attribute(
    std::shared_ptr<wmtk::TetMesh> m,
    const std::string& attr_name,
    int DIM)
{
    const auto* element_block = get_simplex_element_block(DIM);
    const auto tuples = m->get_all(m->top_simplex_type());
    const size_t tag_offset = element_block->data.front();

    for (const auto& data : m_spec.element_data) {
        if (data.header.string_tags.front() != attr_name) {
            continue;
        }
        assert(data.entries.size() == tuples.size());

        const int64_t attr_dim = data.header.int_tags[1];

        auto tag_handle =
            m->register_attribute<double>(attr_name, PrimitiveType::Tetrahedron, attr_dim);
        auto acc = m->create_accessor<double>(tag_handle);

        for (size_t i = 0; i < tuples.size(); ++i) {
            const auto& entry = data.entries[i];
            const size_t tag = entry.tag - tag_offset;
            assert(tag < element_block->num_elements_in_block);
            assert(tag == i);
            if (attr_dim == 1) {
                acc.scalar_attribute(tuples[i]) = entry.data[0];
            } else {
                acc.vector_attribute(tuples[i]) =
                    Eigen::Map<const Eigen::VectorXd>(entry.data.data(), entry.data.size());
            }
        }
        return;
    }
}
} // namespace wmtk