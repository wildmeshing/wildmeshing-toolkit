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

std::unique_ptr<wmtk::Mesh>
extract_subset(wmtk::Mesh& m, long dimension, std::vector<int>& tag_vec, bool pos)
{
    wmtk::PrimitiveType topType = m.top_simplex_type();
    assert(tag_vec.size() == m.capacity(topType));
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
        wmtk::mesh_utils::set_matrix_attribute(tag, "tag", topType, m);
    std::cout << "hello" << std::endl;
    switch (dimension) {
    case 2: {
        if (wmtk::TriMesh* trimesh = dynamic_cast<wmtk::TriMesh*>(&m)) {
            std::unique_ptr<wmtk::Mesh> ret = std::make_unique<wmtk::TriMesh>(
                internal::extract_subset_2d(*trimesh, tag_handle, pos));
            return ret;
        }
        break;
        // return internal::topology_separate_2d(ret);
    }
    case 3: {
        if (wmtk::TetMesh* tetmesh = dynamic_cast<wmtk::TetMesh*>(&m)) {
            std::unique_ptr<wmtk::Mesh> ret = std::make_unique<wmtk::TetMesh>(
                internal::extract_subset_3d(*tetmesh, tag_handle, pos));
            return ret;
        }
        break;
        // return ret;
    }
    default: {
        throw std::runtime_error("Invalid mesh dimension in extracting subset!");
    }
    }
    throw std::runtime_error("Invalid mesh type for the given dimension in extracting subset!");
}

} // namespace components
} // namespace wmtk