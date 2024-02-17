#pragma once

#include <deque>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {

/**
 * @brief This class generates a multi-mesh from a mesh where the substructure is represented by a
 * tag.
 */
class MultiMeshFromTag
{
public:
    MultiMeshFromTag(
        Mesh& mesh,
        const attribute::MeshAttributeHandle& tag_handle,
        const int64_t tag_value);

    void compute_substructure_ids();

private:
    std::vector<Tuple> get_connected_region(const Tuple& t, const PrimitiveType ptype);

private:
    Mesh& m_mesh;

    attribute::MeshAttributeHandle m_tag_handle;
    const attribute::Accessor<int64_t> m_tag_acc;
    const int64_t m_tag_value;
    const PrimitiveType m_tag_ptype;
};

} // namespace wmtk::components::internal
