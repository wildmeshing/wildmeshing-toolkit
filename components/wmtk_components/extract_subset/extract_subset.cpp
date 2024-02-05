#include "extract_subset.hpp"
namespace wmtk {
namespace components {

Eigen::VectorX<long>& vector2tag(Eigen::VectorX<long>& ret, std::vector<int> vector)
{
    ret.resize(vector.size());
    for (int i = 0; i < vector.size(); ++i) ret.row(i) << vector[i];
    return ret;
}

std::unique_ptr<wmtk::Mesh>
extract_subset(wmtk::Mesh& m, const std::vector<int>& tag_vec, bool pos)
{
    wmtk::PrimitiveType topType = m.top_simplex_type();
    // tag vector must have the same size as the number of simplices in the mesh
    assert(tag_vec.size() == m.get_all(topType).size());
    if (pos) { // if user asks to preserve geometry, then geometry must be provided
        try {
            m.get_attribute_handle<double>("position", wmtk::PrimitiveType::Vertex);
        } catch (const std::exception& e) {
            throw std::runtime_error("input mesh doesn't have position attributes!");
        }
    }

    Eigen::VectorX<long> tag;
    wmtk::MeshAttributeHandle<long> tag_handle =
        wmtk::mesh_utils::set_matrix_attribute(vector2tag(tag, tag_vec), "tag", topType, m);

    if (wmtk::TriMesh* trimesh = dynamic_cast<wmtk::TriMesh*>(&m)) {
        std::unique_ptr<wmtk::Mesh> ret =
            std::make_unique<wmtk::TriMesh>(internal::extract_subset_2d(*trimesh, tag_handle, pos));
        return ret;
        // return internal::topology_separate_2d(ret);
    }
    else if (wmtk::TetMesh* tetmesh = dynamic_cast<wmtk::TetMesh*>(&m)) {
        std::unique_ptr<wmtk::Mesh> ret =
            std::make_unique<wmtk::TetMesh>(internal::extract_subset_3d(*tetmesh, tag_handle, pos));
        return ret;
        // return internal::topology_separate_3d(ret);
    } else
        throw std::runtime_error("Invalid mesh dimension in extracting subset!");
}

} // namespace components
} // namespace wmtk