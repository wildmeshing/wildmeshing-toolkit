#pragma once

#include <functional>
#include <iostream>
#include <optional>
#include <string>

#include <mshio/mshio.h>
#include <wmtk/Types.hpp>

#include "Logger.hpp"

namespace wmtk {

/**
 * Write meshes to a .msh file. The right order of calling functions is important!
 *
 * 1. Add vertices. Creates a new entity.
 * 2. Add faces/tets. Adds faces/tets with the same entity tag as the last added vertex block. If the vertex block was empty, the vertex IDs in the elements will be considered global (no offset is applied to the IDs).
 * 3. Add attributes and physical groups.
 */
class MshData
{
public:
    template <typename Fn>
    void add_edge_vertices(size_t num_vertices, const Fn& get_vertex_cb)
    {
        add_vertices<1>(num_vertices, get_vertex_cb);
    }

    /**
     * @brief Add vertex block. Must be called before adding anything else.
     */
    template <typename Fn>
    void add_face_vertices(size_t num_vertices, const Fn& get_vertex_cb)
    {
        add_vertices<2>(num_vertices, get_vertex_cb);
    }

    template <typename Fn>
    void add_tet_vertices(size_t num_vertices, const Fn& get_vertex_cb)
    {
        add_vertices<3>(num_vertices, get_vertex_cb);
    }

    void add_empty_vertices(int dim)
    {
        if (dim < 1 || dim > 3) {
            log_and_throw_error("Only 1,2,3D elements are supported!");
        }

        if (m_spec.nodes.entity_blocks.empty()) {
            log_and_throw_error("First vertex block cannot be empty.");
        }

        mshio::NodeBlock block;
        block.num_nodes_in_block = 0;
        block.entity_dim = dim;
        block.entity_tag = m_spec.nodes.num_entity_blocks + 1;

        m_spec.nodes.num_entity_blocks += 1;
        m_spec.nodes.entity_blocks.push_back(std::move(block));
    }

    void add_edge_vertices() { add_empty_vertices(1); }
    void add_face_vertices() { add_empty_vertices(2); }
    void add_tet_vertices() { add_empty_vertices(3); }

    template <typename Fn>
    void add_edges(size_t num_edges, const Fn& get_edge_cb)
    {
        add_simplex_elements<1>(num_edges, get_edge_cb);
    }

    /**
     * @brief Add face block. Must be called after vertices were added.
     *
     * The face block will get the same entity tag as the previously added vertex block.
     *
     * @param num_faces Number of faces.
     * @param get_face_cb Callback function for getting a face by ID.
     */
    template <typename Fn>
    void add_faces(size_t num_faces, const Fn& get_face_cb)
    {
        add_simplex_elements<2>(num_faces, get_face_cb);
    }

    template <typename Fn>
    void add_tets(size_t num_tets, const Fn& get_tet_cb)
    {
        add_simplex_elements<3>(num_tets, get_tet_cb);
    }

    template <int NUM_FIELDS, typename Fn>
    void add_edge_vertex_attribute(const std::string& name, const Fn& get_attribute_cb)
    {
        add_vertex_attribute<NUM_FIELDS, 1>(name, get_attribute_cb);
    }

    template <int NUM_FIELDS, typename Fn>
    void add_face_vertex_attribute(const std::string& name, const Fn& get_attribute_cb)
    {
        add_vertex_attribute<NUM_FIELDS, 2>(name, get_attribute_cb);
    }

    template <int NUM_FIELDS, typename Fn>
    void add_tet_vertex_attribute(const std::string& name, const Fn& get_attribute_cb)
    {
        add_vertex_attribute<NUM_FIELDS, 3>(name, get_attribute_cb);
    }

    template <int NUM_FIELDS, typename Fn>
    void add_edge_attribute(const std::string& name, const Fn& get_attribute_cb)
    {
        add_element_attribute<NUM_FIELDS, 1>(name, get_attribute_cb);
    }

    template <int NUM_FIELDS, typename Fn>
    void add_face_attribute(const std::string& name, const Fn& get_attribute_cb)
    {
        add_element_attribute<NUM_FIELDS, 2>(name, get_attribute_cb);
    }

    template <int NUM_FIELDS, typename Fn>
    void add_tet_attribute(const std::string& name, const Fn& get_attribute_cb)
    {
        add_element_attribute<NUM_FIELDS, 3>(name, get_attribute_cb);
    }

    /**
     * @brief Adds the pysical group and the corresponding entity entry.
     */
    void add_physical_group(const std::string& name)
    {
        auto& entities = m_spec.entities;

        const auto& vertex_block = m_spec.nodes.entity_blocks.back();
        const int tag = vertex_block.entity_tag;
        const int dim = vertex_block.entity_dim;

        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double min_z = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();
        double max_z = std::numeric_limits<double>::lowest();
        for (size_t i = 0; i < vertex_block.num_nodes_in_block; ++i) {
            min_x = std::min(vertex_block.data[3 * i + 0], min_x);
            min_y = std::min(vertex_block.data[3 * i + 1], min_y);
            min_z = std::min(vertex_block.data[3 * i + 2], min_z);
            max_x = std::max(vertex_block.data[3 * i + 0], max_x);
            max_y = std::max(vertex_block.data[3 * i + 1], max_y);
            max_z = std::max(vertex_block.data[3 * i + 2], max_z);
        }

        mshio::PhysicalGroup ph;
        ph.dim = dim;
        ph.tag = tag;
        ph.name = name;
        m_spec.physical_groups.emplace_back(ph);

        switch (dim) {
        case 0: {
            mshio::PointEntity e{tag, min_x, min_y, min_z};
            e.physical_group_tags.push_back(tag);
            entities.points.push_back(e);
            break;
        }
        case 1: {
            mshio::CurveEntity e{tag, min_x, min_y, min_z, max_x, max_y, max_z};
            e.physical_group_tags.push_back(tag);
            entities.curves.push_back(e);
            break;
        }
        case 2: {
            mshio::SurfaceEntity e{tag, min_x, min_y, min_z, max_x, max_y, max_z};
            e.physical_group_tags.push_back(tag);
            entities.surfaces.push_back(e);
            break;
        }
        case 3: {
            mshio::VolumeEntity e{tag, min_x, min_y, min_z, max_x, max_y, max_z};
            e.physical_group_tags.push_back(tag);
            entities.volumes.push_back(e);
            break;
        }
        default: log_and_throw_error("Entity of dim {} cannot be written to .msh.");
        }
    }

    inline size_t get_num_edge_vertices() const { return get_num_vertices(1); }

    inline size_t get_num_face_vertices() const { return get_num_vertices(2); }

    inline size_t get_num_tet_vertices() const { return get_num_vertices(3); }

    inline size_t get_num_edges() const { return get_num_simplex_elements(1); }

    inline size_t get_num_faces() const { return get_num_simplex_elements(2); }

    inline size_t get_num_tets() const { return get_num_simplex_elements(3); }

    template <typename Fn>
    void extract_tet_vertices(Fn&& set_vertex_cb) const
    {
        return extract_vertices(3, std::forward<Fn>(set_vertex_cb));
    }

    template <typename Fn>
    void extract_face_vertices(Fn&& set_vertex_cb) const
    {
        return extract_vertices(2, std::forward<Fn>(set_vertex_cb));
    }

    template <typename Fn>
    void extract_edge_vertices(Fn&& set_vertex_cb) const
    {
        extract_vertices(1, std::forward<Fn>(set_vertex_cb));
    }

    template <typename Fn>
    void extract_edges(Fn&& set_edge_cb) const
    {
        extract_simplex_elements<1>(std::forward<Fn>(set_edge_cb));
    }

    template <typename Fn>
    void extract_faces(Fn&& set_face_cb) const
    {
        extract_simplex_elements<2>(std::forward<Fn>(set_face_cb));
    }

    template <typename Fn>
    void extract_tets(Fn&& set_tet_cb) const
    {
        extract_simplex_elements<3>(std::forward<Fn>(set_tet_cb));
    }

    std::vector<std::string> get_edge_vertex_attribute_names() const
    {
        return get_vertex_attribute_names<1>();
    }

    std::vector<std::string> get_face_vertex_attribute_names() const
    {
        return get_vertex_attribute_names<2>();
    }

    std::vector<std::string> get_tet_vertex_attribute_names() const
    {
        return get_vertex_attribute_names<3>();
    }

    std::vector<std::string> get_edge_attribute_names() const
    {
        return get_element_attribute_names<1>();
    }

    std::vector<std::string> get_face_attribute_names() const
    {
        return get_element_attribute_names<2>();
    }

    std::vector<std::string> get_tet_attribute_names() const
    {
        return get_element_attribute_names<3>();
    }

    template <typename Fn>
    void extract_edge_vertex_attribute(const std::string& attr_name, Fn&& set_attr)
    {
        extract_vertex_attribute<1>(attr_name, std::forward<Fn>(set_attr));
    }

    template <typename Fn>
    void extract_face_vertex_attribute(const std::string& attr_name, Fn&& set_attr)
    {
        extract_vertex_attribute<2>(attr_name, std::forward<Fn>(set_attr));
    }

    template <typename Fn>
    void extract_tet_vertex_attribute(const std::string& attr_name, Fn&& set_attr)
    {
        extract_vertex_attribute<3>(attr_name, std::forward<Fn>(set_attr));
    }

    template <typename Fn>
    void extract_edge_attribute(const std::string& attr_name, Fn&& set_attr)
    {
        extract_element_attribute<1>(attr_name, std::forward<Fn>(set_attr));
    }

    template <typename Fn>
    void extract_face_attribute(const std::string& attr_name, Fn&& set_attr)
    {
        extract_element_attribute<2>(attr_name, std::forward<Fn>(set_attr));
    }

    template <typename Fn>
    void extract_tet_attribute(const std::string& attr_name, Fn&& set_attr)
    {
        extract_element_attribute<3>(attr_name, std::forward<Fn>(set_attr));
    }

    /**
     * @brief Returns the (first) physical group with the given name, if it exists.
     */
    std::optional<mshio::PhysicalGroup> get_physical_group_by_name(const std::string& name)
    {
        for (const mshio::PhysicalGroup& ph : m_spec.physical_groups) {
            if (ph.name == name) {
                return ph;
            }
        }
        return {};
    }

    void get_VF(const int& dim, const int& tag, MatrixXd& V, MatrixXi& F)
    {
        assert(dim > 0 && tag > 0);

        const size_t n_V = get_num_vertices(dim, tag);
        const size_t n_F = get_num_simplex_elements(dim, tag);

        if (n_V > 0) {
            V.resize(n_V, 3);
            extract_vertices(dim, tag, [&V](size_t i, double x, double y, double z) {
                V.row(i) = Vector3d(x, y, z);
            });
        }
        if (n_F > 0) {
            F.resize(n_F, dim + 1);
            switch (dim) {
            case 1:
                extract_simplex_elements<1>(
                    [&F](size_t i, size_t v0, size_t v1) { F.row(i) = Vector2i(v0, v1); });
                break;
            case 2:
                extract_simplex_elements<2>([&F](size_t i, size_t v0, size_t v1, size_t v2) {
                    F.row(i) = Vector3i(v0, v1, v2);
                });
                break;
            case 3:
                extract_simplex_elements<3>(
                    [&F](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
                        F.row(i) = Vector4i(v0, v1, v2, v3);
                    });
                break;
            default: log_and_throw_error("Cannot extract elements for dimension {}", dim);
            }
        }
    }

    void save(const std::string& filename, bool binary = true)
    {
        m_spec.mesh_format.file_type = binary;
        mshio::validate_spec(m_spec);
        mshio::save_msh(filename, m_spec);
    }

    void save(std::ostream& out, bool binary = true)
    {
        m_spec.mesh_format.file_type = binary;
        mshio::validate_spec(m_spec);
        mshio::save_msh(out, m_spec);
    }

    void load(const std::string& filename) { m_spec = mshio::load_msh(filename); }

    void load(std::istream& in) { m_spec = mshio::load_msh(in); }

private:
    template <int DIM, typename Fn>
    void add_vertices(size_t num_vertices, const Fn& get_vertex_cb)
    {
        static_assert(DIM >= 1 && DIM <= 3, "Only 1,2,3D elements are supported!");
        if (num_vertices == 0) {
            logger().trace("Adding empty vertex block.");
        }
        mshio::NodeBlock block;
        block.num_nodes_in_block = num_vertices;
        block.tags.reserve(num_vertices);
        block.data.reserve(num_vertices * 3);
        block.entity_dim = DIM;
        block.entity_tag = m_spec.nodes.num_entity_blocks + 1;

        const size_t tag_offset = m_spec.nodes.max_node_tag;
        for (size_t i = 0; i < num_vertices; i++) {
            const auto& v = get_vertex_cb(i);
            block.tags.push_back(tag_offset + i + 1);
            block.data.push_back(v[0]);
            block.data.push_back(v[1]);
            block.data.push_back(v[2]);
        }

        m_spec.nodes.num_entity_blocks += 1;
        m_spec.nodes.num_nodes += num_vertices;
        m_spec.nodes.min_node_tag = 1;
        m_spec.nodes.max_node_tag += num_vertices;
        m_spec.nodes.entity_blocks.push_back(std::move(block));
    }

    template <int DIM, typename Fn>
    void add_simplex_elements(size_t num_elements, const Fn& get_element_cb)
    {
        static_assert(
            DIM == 1 || DIM == 2 || DIM == 3,
            "Only 1,2,3D simplex elements are supported");
        if (num_elements == 0) return;

        if (m_spec.nodes.num_nodes == 0) {
            log_and_throw_error("Please add a vertex block before adding elements.");
        }
        const auto& vertex_block = m_spec.nodes.entity_blocks.back();
        // assert(!vertex_block.tags.empty());
        if (vertex_block.entity_dim != DIM) {
            log_and_throw_error("It seems the last added vertex block has different dimension "
                                "than the elements you want to add.");
        }

        mshio::ElementBlock block;
        block.entity_dim = DIM;
        block.entity_tag = vertex_block.entity_tag;
        if constexpr (DIM == 1) {
            block.element_type = 1; // 2-node line.
        } else if constexpr (DIM == 2) {
            block.element_type = 2; // 3-node triangle.
        } else {
            block.element_type = 4; // 4-node tet.
        }
        block.num_elements_in_block = num_elements;

        const size_t vertex_offset =
            vertex_block.num_nodes_in_block == 0 ? 0 : vertex_block.tags.front() - 1;
        if (vertex_offset == 0) {
            logger().trace(
                "Do not offset vertex IDs for this element block as vertex block was empty");
        }

        const size_t tag_offset = m_spec.elements.max_element_tag;
        block.data.reserve(num_elements * (DIM + 2));
        for (size_t i = 0; i < num_elements; i++) {
            const auto& e = get_element_cb(i);
            block.data.push_back(tag_offset + i + 1); // element tag.
            for (size_t j = 0; j <= DIM; j++) {
                block.data.push_back(vertex_offset + e[j] + 1);
            }
        }

        m_spec.elements.num_entity_blocks++;
        m_spec.elements.num_elements += num_elements;
        m_spec.elements.min_element_tag = 1;
        m_spec.elements.max_element_tag += num_elements;
        m_spec.elements.entity_blocks.push_back(std::move(block));
    }

    template <int NUM_FIELDS, int ELEMENT_DIM, typename Fn>
    void add_vertex_attribute(const std::string& name, const Fn& get_attribute_cb)
    {
        static_assert(
            NUM_FIELDS == 1 || NUM_FIELDS == 3 || NUM_FIELDS == 9,
            "Only scalar, vector and tensor fields are supported as attribute!");
        if (m_spec.nodes.entity_blocks.empty()) {
            throw std::runtime_error("Please add vertices before adding vertex attributes.");
        }
        const auto& vertex_block = m_spec.nodes.entity_blocks.back();
        const size_t num_vertices = vertex_block.num_nodes_in_block;
        assert(num_vertices != 0);

        if (vertex_block.entity_dim != ELEMENT_DIM) {
            throw std::runtime_error("It seems the last added vertex block has different dimension "
                                     "from the vertex attribute you want to add.");
        }

        mshio::Data data;
        data.header.string_tags = {name};
        data.header.real_tags = {0.0};
        data.header.int_tags = {0, NUM_FIELDS, int(num_vertices), 0, ELEMENT_DIM};

        data.entries.resize(num_vertices);
        for (size_t i = 0; i < num_vertices; i++) {
            auto& entry = data.entries[i];
            entry.tag = vertex_block.tags[i];
            entry.data.reserve(NUM_FIELDS);
            const auto& attr = get_attribute_cb(i);
            if constexpr (NUM_FIELDS == 1) {
                entry.data.push_back(attr);
            } else {
                for (size_t j = 0; j < NUM_FIELDS; j++) {
                    entry.data.push_back(attr[j]);
                }
            }
        }

        m_spec.node_data.push_back(std::move(data));
    }

    template <int NUM_FIELDS, int ELEMENT_DIM, typename Fn>
    void add_element_attribute(const std::string& name, const Fn& get_attribute_cb)
    {
        static_assert(
            NUM_FIELDS == 1 || NUM_FIELDS == 3 || NUM_FIELDS == 9,
            "Only scalar, vector and tensor fields are supported as attribute!");
        if (m_spec.elements.entity_blocks.empty()) {
            throw std::runtime_error("Please add elements before adding element attributes!");
        }
        const auto& elem_block = m_spec.elements.entity_blocks.back();
        if (elem_block.entity_dim != ELEMENT_DIM) {
            throw std::runtime_error(
                "It seems the last added element block has different dimension "
                "than the element attribute you want to add.");
        }
        const size_t num_elements = elem_block.num_elements_in_block;

        mshio::Data data;
        data.header.string_tags = {name};
        data.header.real_tags = {0.0};
        data.header.int_tags = {0, NUM_FIELDS, int(num_elements), 0, ELEMENT_DIM};

        data.entries.resize(num_elements);
        for (size_t i = 0; i < num_elements; i++) {
            auto& entry = data.entries[i];
            entry.tag = elem_block.data[i * (ELEMENT_DIM + 2)];
            entry.data.reserve(NUM_FIELDS);
            const auto& attr = get_attribute_cb(i);
            if constexpr (NUM_FIELDS == 1) {
                entry.data.push_back(attr);
            } else {
                for (size_t j = 0; j < NUM_FIELDS; j++) {
                    entry.data.push_back(attr[j]);
                }
            }
        }

        m_spec.element_data.push_back(std::move(data));
    }

    const mshio::NodeBlock* get_vertex_block(const int dim) const
    {
        for (const auto& block : m_spec.nodes.entity_blocks) {
            if (block.entity_dim == dim) {
                return &block;
            }
        }
        return nullptr;
    }

    const mshio::NodeBlock* get_vertex_block(const int dim, const int tag) const
    {
        for (const auto& block : m_spec.nodes.entity_blocks) {
            if (block.entity_dim == dim && block.entity_tag == tag) {
                return &block;
            }
        }
        return nullptr;
    }

    const mshio::ElementBlock* get_simplex_element_block(const int dim) const
    {
        for (const auto& block : m_spec.elements.entity_blocks) {
            if (block.entity_dim == dim) {
                return &block;
            }
        }
        return nullptr;
    }

    const mshio::ElementBlock* get_simplex_element_block(const int dim, const int tag) const
    {
        for (const auto& block : m_spec.elements.entity_blocks) {
            if (block.entity_dim == dim && block.entity_tag == tag) {
                return &block;
            }
        }
        return nullptr;
    }

    size_t get_num_vertices(const int dim) const
    {
        const auto* block = get_vertex_block(dim);
        if (block != nullptr) {
            return block->num_nodes_in_block;
        } else {
            return 0;
        }
    }

    size_t get_num_vertices(const int dim, const int tag) const
    {
        const auto* block = get_vertex_block(dim, tag);
        if (block != nullptr) {
            return block->num_nodes_in_block;
        } else {
            return 0;
        }
    }

    size_t get_num_simplex_elements(const int dim) const
    {
        const auto* block = get_simplex_element_block(dim);
        if (block != nullptr) {
            return block->num_elements_in_block;
        } else {
            return 0;
        }
    }

    size_t get_num_simplex_elements(const int dim, const int tag) const
    {
        const auto* block = get_simplex_element_block(dim, tag);
        if (block != nullptr) {
            return block->num_elements_in_block;
        } else {
            return 0;
        }
    }

    template <typename Fn>
    void extract_vertices(const int dim, Fn&& set_vertex_cb) const
    {
        const auto* block = get_vertex_block(dim);
        if (block == nullptr) return;

        const size_t num_vertices = block->num_nodes_in_block;
        if (num_vertices == 0) return;

        const size_t tag_offset = block->tags.front();
        for (size_t i = 0; i < num_vertices; i++) {
            size_t tag = block->tags[i] - tag_offset;
            set_vertex_cb(tag, block->data[i * 3], block->data[i * 3 + 1], block->data[i * 3 + 2]);
        }
    }

    template <typename Fn>
    void extract_vertices(const int dim, const int tag, Fn&& set_vertex_cb) const
    {
        const auto* block = get_vertex_block(dim, tag);
        if (block == nullptr) return;

        const size_t num_vertices = block->num_nodes_in_block;
        if (num_vertices == 0) return;

        const size_t tag_offset = block->tags.front();
        for (size_t i = 0; i < num_vertices; i++) {
            size_t tag = block->tags[i] - tag_offset;
            set_vertex_cb(tag, block->data[i * 3], block->data[i * 3 + 1], block->data[i * 3 + 2]);
        }
    }

    template <int DIM, typename Fn>
    void extract_simplex_elements(Fn&& set_element_cb) const
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
                set_element_cb(tag, element[0] - vert_tag_offset, element[1] - vert_tag_offset);
            } else if constexpr (DIM == 2) {
                set_element_cb(
                    tag,
                    element[0] - vert_tag_offset,
                    element[1] - vert_tag_offset,
                    element[2] - vert_tag_offset);
            } else if constexpr (DIM == 3) {
                set_element_cb(
                    tag,
                    element[0] - vert_tag_offset,
                    element[1] - vert_tag_offset,
                    element[2] - vert_tag_offset,
                    element[3] - vert_tag_offset);
            }
        }
    }

    template <int DIM>
    std::vector<std::string> get_vertex_attribute_names() const
    {
        std::vector<std::string> attr_names;
        attr_names.reserve(m_spec.node_data.size());
        for (const auto& data : m_spec.node_data) {
            const auto& int_tags = data.header.int_tags;
            if (int_tags.size() >= 5 && int_tags[4] == DIM) {
                attr_names.push_back(data.header.string_tags.front());
            }
        }
        return attr_names;
    }

    template <int DIM>
    std::vector<std::string> get_element_attribute_names() const
    {
        std::vector<std::string> attr_names;
        attr_names.reserve(m_spec.element_data.size());
        for (const auto& data : m_spec.element_data) {
            const auto& int_tags = data.header.int_tags;
            if (int_tags.size() >= 5 && int_tags[4] == DIM) {
                attr_names.push_back(data.header.string_tags.front());
            }
        }
        return attr_names;
    }

    template <int DIM, typename Fn>
    void extract_vertex_attribute(const std::string& attr_name, Fn&& set_attr)
    {
        const auto* vertex_block = get_vertex_block(DIM);
        const size_t tag_offset = vertex_block->tags.front();
        for (const auto& data : m_spec.node_data) {
            if (data.header.string_tags.front() != attr_name) continue;
            if (data.header.int_tags.size() >= 5 && data.header.int_tags[4] != DIM) {
                log_and_throw_error("Attribute " + attr_name + " is of the wrong DIM.");
            }

            for (const auto& entry : data.entries) {
                const size_t tag = entry.tag - tag_offset;
                assert(tag < vertex_block->num_nodes_in_block);
                set_attr(tag, entry.data);
            }
        }
    }

    template <int DIM, typename Fn>
    void extract_element_attribute(const std::string& attr_name, Fn&& set_attr)
    {
        const auto* element_block = get_simplex_element_block(DIM);
        const size_t tag_offset = element_block->data.front();
        for (const auto& data : m_spec.element_data) {
            if (data.header.string_tags.front() != attr_name) continue;
            if (data.header.int_tags.size() >= 5 && data.header.int_tags[4] != DIM) {
                throw std::runtime_error("Attribute " + attr_name + " is of the wrong DIM.");
            }

            for (const mshio::DataEntry& entry : data.entries) {
                const size_t tag = entry.tag - tag_offset;
                assert(tag < element_block->num_elements_in_block);
                set_attr(tag, entry.data);
            }
        }
    }


public:
    mshio::MshSpec m_spec;
};

} // namespace wmtk
