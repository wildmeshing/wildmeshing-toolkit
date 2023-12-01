#include "extract_subset.hpp"
namespace wmtk {
namespace components {


template <typename T>
Eigen::VectorX<T>& vector2tag(Eigen::VectorX<T>& ret, std::vector<int> vector)
{
    ret.resize(vector.size());
    for (int i = 0; i < vector.size(); ++i) {
        ret.row(i) << vector[i];
    }
    return ret;
}

wmtk::TriMesh extract_subset(wmtk::TriMesh m, long dimension, std::vector<int>& tag_vec, bool pos)
{
    assert(tag_vec.size() == m.capacity(wmtk::PrimitiveType::Face));
    if (pos) { // if user asks to preserve geometry, then geometry must be provided
        try {
            m.get_attribute_handle<double>("position", wmtk::PrimitiveType::Vertex);
        } catch (const std::exception& e) {
            throw std::runtime_error("input mesh doesn't have position attributes!");
        }
    }

    Eigen::VectorX<long> tag;
    tag = vector2tag(tag, tag_vec);
    wmtk::MeshAttributeHandle<long> tag_handle =
        wmtk::mesh_utils::set_matrix_attribute(tag, "tag", wmtk::PrimitiveType::Face, m);
    switch (dimension) {
    case 2: {
        wmtk::TriMesh ret = internal::extract_subset_2d(m, tag_handle, pos);
        return ret;
        // return internal::topology_separate_2d(ret);
    }
    case 3: {
        // to be implemented
        throw std::runtime_error("not implemented");
    }
    default: {
        // to be implemented
        throw std::runtime_error("not implemented");
    }
    }
}

} // namespace components
} // namespace wmtk