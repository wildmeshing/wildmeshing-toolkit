#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components {

/**
 * This component implements a marching triangle/tetrahedra. The method solely relies on edge splits.
 */
class Marching
{
public:
    /**
     * @brief Initialize the marching method.
     *
     * @param pos_handle The position attribute handle.
     * @param label_handles A map from PrimitiveType to the according label attribute. An attribute
     * must be given for at least the vertices.
     */
    Marching(
        attribute::MeshAttributeHandle& pos_handle,
        std::map<PrimitiveType, attribute::MeshAttributeHandle>& label_handles,
        const std::vector<int64_t>& input_values,
        const int64_t output_value);

    /**
     * @brief Perform the actual marching.
     *
     * This method registers new attributes that are not automatically cleaned up!
     */
    void process();

    /**
     * @brief Add an edge filter to marching.
     *
     * Only edges that contain all filters may be split. If no filter exists, all edges are
     * considered for splitting.
     *
     * This filter criterium can be inverted.
     */
    void add_filter(const attribute::MeshAttributeHandle& label, const int64_t value);

    /**
     * @brief Add pass through attributes.
     */
    void add_pass_through(const attribute::MeshAttributeHandle& pass_through);

    /**
     * @brief Add pass through attributes.
     */
    void add_pass_through(const std::vector<attribute::MeshAttributeHandle>& pass_through);

    /**
     * @brief Position the new vertices along an edge according to an isovalue in a scalar field.
     *
     * Marching will try to match the isovalue for each new vertex. However, flipped triangles or
     * vertices are prohibited. A binary search is applied to find a good position if the isovalue
     * cannot be interpolated directly. If the binary search fails, the midpoint of the edge is
     * chosen as position.
     *
     * @param scalar_field Vertex attribute of type double.
     * @param isovalue The isovalue which is interpolated for the new vertices.
     */
    void add_isovalue(const attribute::MeshAttributeHandle& scalar_field, const double isovalue);

    /**
     * @brief Set the number of iterations for the linesearch when trying to match the isovalue.
     *
     * This only has an effect if an isovalue was added.
     * The default for linesearch is 10.
     */
    void set_isovalue_linesearch_iterations(const int64_t linesearch_iterations);

    /**
     * @brief Invert the filter such that everything covered by the filter is ignored.
     */
    void invert_filter();

private:
    Mesh& m_mesh;

    attribute::MeshAttributeHandle m_pos_handle;
    attribute::MeshAttributeHandle m_vertex_tag_handle;
    std::optional<attribute::MeshAttributeHandle> m_edge_tag_handle;
    std::optional<attribute::MeshAttributeHandle> m_face_tag_handle;
    std::vector<int64_t> m_input_values;
    int64_t m_output_value;

    std::vector<attribute::MeshAttributeHandle> m_filter_labels;
    std::vector<int64_t> m_filter_values;

    std::vector<attribute::MeshAttributeHandle> m_pass_through_attributes;

    std::optional<attribute::MeshAttributeHandle> m_scalar_field;
    double m_isovalue = std::numeric_limits<double>::lowest();

    int64_t m_linesearch_iterations = 10;

    bool m_invert_filter = false;
};

} // namespace wmtk::components
