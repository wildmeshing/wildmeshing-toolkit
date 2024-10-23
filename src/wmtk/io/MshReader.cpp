#include "MshReader.hpp"
#include <wmtk/utils/mesh_type_from_primitive_type.hpp>

#include <set>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk/utils/orient.hpp>

#include "mshio/MshSpec.h"
#include "predicates.h"

namespace wmtk::io {
MshReader::MshReader() = default;
MshReader::~MshReader() = default;

template <int DIM>
void MshReader::extract()
{
    extract_vertices<DIM>();
    extract_simplex_elements<DIM>();
}
int MshReader::get_mesh_dimension() const
{
    for (int dim = 3; dim > 0; --dim) {
        if (get_num_simplex_elements(dim) > 0) {
            return dim;
        }
    }
    return 0;
}

std::shared_ptr<Mesh> MshReader::read(
    const std::filesystem::path& filename,
    const bool ignore_z_if_zero,
    const std::vector<std::string>& attrs)
{
    m_spec = mshio::load_msh(filename.string());
    m_embedded_dimension = ignore_z_if_zero ? AUTO_EMBEDDED_DIMENSION : 3;

    std::vector<std::vector<std::string>> split_attrs(get_mesh_dimension() + 1);

    split_attrs.back() = attrs;

    return generate(split_attrs);
}
// reading from an msh file while preserve the specified attributes, only supports valid scalar
// attributes on tets in a tetmesh
std::shared_ptr<Mesh> MshReader::read(
    const std::filesystem::path& filename,
    const int64_t embedded_dimension,
    const std::vector<std::vector<std::string>>& extra_attributes)
{
    m_spec = mshio::load_msh(filename.string());
    m_embedded_dimension = embedded_dimension == -1 ? get_embedded_dimension() : embedded_dimension;


    return generate(extra_attributes);
}

std::shared_ptr<Mesh> MshReader::read(
    const std::filesystem::path& filename,
    const int64_t embedded_dimension)
{
    m_spec = mshio::load_msh(filename.string());
    m_embedded_dimension = embedded_dimension == -1 ? get_embedded_dimension() : embedded_dimension;


    return generate();
}


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

size_t MshReader::get_num_vertices(int DIM) const
{
    const auto* block = get_vertex_block(DIM);
    if (block != nullptr) {
        return block->num_nodes_in_block;
    } else {
        return 0;
    }
}

size_t MshReader::get_num_simplex_elements(int DIM) const
{
    const auto* block = get_simplex_element_block(DIM);
    if (block != nullptr) {
        return block->num_elements_in_block;
    } else {
        return 0;
    }
}


int MshReader::get_embedded_dimension() const
{
    const mshio::NodeBlock* block = get_vertex_block(get_mesh_dimension());
    assert(block != nullptr);

    return block->entity_dim;
}

template <int DIM>
void MshReader::extract_vertices()
{
    const mshio::NodeBlock* block = get_vertex_block(DIM);
    assert(block != nullptr);

    const size_t num_vertices = block->num_nodes_in_block;
    const auto embedded_dimension =
        m_embedded_dimension == AUTO_EMBEDDED_DIMENSION ? 3 : m_embedded_dimension;
    V.resize(num_vertices, embedded_dimension);

    // previously this code returned if false, but later on there is an assumption the vertex block
    // is filled out  so it is now an assertion
    assert(num_vertices > 0);

    assert(embedded_dimension <= 3);
    assert(embedded_dimension > 0);

    using Vector = wmtk::Vector<double, 3>;
    using ConstMapType = typename Vector::ConstMapType;

    const size_t tag_offset = block->tags.front();
    for (size_t i = 0; i < num_vertices; i++) {
        size_t tag = block->tags[i] - tag_offset;
        ConstMapType vec(block->data.data() + i * 3);
        V.row(tag) = vec.head(embedded_dimension).transpose();
    }

    if (m_embedded_dimension == AUTO_EMBEDDED_DIMENSION) {
        const double max = V.col(2).array().maxCoeff();
        const double min = V.col(2).array().minCoeff();

        if (min == 0 && max == 0) {
            V = V.leftCols(2);
        }
    }
}

template <int DIM>
void MshReader::extract_simplex_elements()
{
    const mshio::NodeBlock* vertex_block = get_vertex_block(DIM);
    const mshio::ElementBlock* element_block = get_simplex_element_block(DIM);
    if (element_block == nullptr) return;
    assert(vertex_block != nullptr);

    const size_t num_elements = element_block->num_elements_in_block;
    if (num_elements == 0) return;
    assert(vertex_block->num_nodes_in_block != 0);

    S.resize(num_elements, DIM + 1);

    const size_t vert_tag_offset = vertex_block->tags.front();
    const size_t elem_tag_offset = element_block->data.front();
    for (size_t i = 0; i < num_elements; i++) {
        const size_t tag = element_block->data[i * (DIM + 2)] - elem_tag_offset;
        assert(tag < num_elements);
        const size_t* element = element_block->data.data() + i * (DIM + 2) + 1;

        using Vector = wmtk::Vector<size_t, DIM + 1>;
        using ConstMapType = typename Vector::ConstMapType;

        ConstMapType element_vec(element);
        const auto vertex_tag_offset_vec = Vector::Constant(vert_tag_offset);

        S.row(tag) = (element_vec - vertex_tag_offset_vec).transpose().template cast<int64_t>();
    }
}
template <int DIM>
auto MshReader::construct() -> std::shared_ptr<wmtk::utils::mesh_type_from_dimension_t<DIM>>
{
    using MeshType = wmtk::utils::mesh_type_from_dimension_t<DIM>;
    auto ret = std::make_shared<MeshType>();
    ret->initialize(S);
    return ret;
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
    Mesh& m,
    const std::string& attr_name,
    PrimitiveType primitive_type)
{
    assert(primitive_type == m.top_simplex_type());
    const int64_t primitive_dimension = get_primitive_type_id(primitive_type);
    const mshio::ElementBlock* element_block = get_simplex_element_block(primitive_dimension);
    const auto tuples = m.get_all(m.top_simplex_type());
    const size_t tag_offset = element_block->data.front();

    for (const auto& data : m_spec.element_data) {
        if (data.header.string_tags.front() != attr_name) {
            continue;
        }
        assert(data.entries.size() == tuples.size());

        const int64_t attr_dim = data.header.int_tags[1];

        auto tag_handle = m.register_attribute<double>(attr_name, primitive_type, attr_dim);
        auto acc = m.create_accessor<double>(tag_handle);

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

auto MshReader::generate(
    const std::optional<std::vector<std::vector<std::string>>>& extra_attributes_opt)
    -> std::shared_ptr<Mesh>
{
    std::shared_ptr<Mesh> res;
    switch (get_mesh_dimension()) {
    case 3: res = generateT<3>(); break;
    case 2: res = generateT<2>(); break;
    case 1: res = generateT<1>(); break;
    default:
    case 0: res = std::make_shared<PointMesh>();
    }
    assert(V.size() > 0);

    if (extra_attributes_opt.has_value()) {
        const auto& extra_attributes = extra_attributes_opt.value();
        // save all valid scalar attributes on tets
        std::set<std::string> valid_attr_names;
        for (const auto& data : m_spec.element_data) {
            // validness: int_tags[2] stores the dimension of an element_data, which should
            // match the size of tets
            if (data.header.int_tags[2] != get_num_simplex_elements(3)) {
                continue;
            }
            valid_attr_names.insert(data.header.string_tags.front());
        }
        for (size_t dim = 0; dim < extra_attributes.size(); ++dim) {
            const auto& attr_names = extra_attributes[dim];
            for (const std::string& attr : attr_names) {
                if (valid_attr_names.count(attr) == 0) {
                    log_and_throw_error("Attribute " + attr + " does not exist in the msh file.");
                }
                extract_element_attribute(*res, attr, get_primitive_type_from_id(dim));
            }
        }
    }
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *res);
    return res;
}

template <int DIM>
auto MshReader::generateT() -> std::shared_ptr<wmtk::utils::mesh_type_from_dimension_t<DIM>>
{
    extract<DIM>();
    validate<DIM>();
    return construct<DIM>();
}

template <int DIM>
void MshReader::validate()
{}

template <>
void MshReader::validate<3>()
{
    assert(V.cols() == 3);
    exactinit();
    {
        // check inversion

        auto Vs = V(S.row(0), Eigen::all);


        if (wmtk::utils::wmtk_orient3d(Vs.transpose()) < 0) {
            // swap col 0 and 1 of S
            S.col(0).swap(S.col(1));
            wmtk::logger().info(
                "Input tet orientation is inverted, swapping col 0 and 1 of TV matirx.");
        }
    }

    {
        // check consistency
        for (int64_t i = 0; i < S.rows(); i++) {
            auto row = S.row(i);
            auto Vs = V(row, Eigen::all);
            auto orient = wmtk::utils::wmtk_orient3d(Vs.transpose());

            if (orient < 0) {
                std::swap(row.x(), row.y());
                // log_and_throw_error("Input tet orientation is inconsistent.");
            } else if (orient == 0) {
                log_and_throw_error("Input tet is degenerated.");
            }
        }
    }

    {
        // check consistency
        for (int64_t i = 0; i < S.rows(); i++) {
            auto row = S.row(i);
            auto Vs = V(row, Eigen::all);
            auto orient = wmtk::utils::wmtk_orient3d(Vs.transpose());

            if (orient < 0) {
                auto a = Vs.row(0);
                auto b = Vs.row(1);
                auto tmp = a.eval();
                a = b;
                b = a;
                // log_and_throw_error("Input tet orientation is inconsistent.");
            } else if (orient == 0) {
                log_and_throw_error("Input tet is degenerated.");
            }
        }
    }
}
} // namespace wmtk::io