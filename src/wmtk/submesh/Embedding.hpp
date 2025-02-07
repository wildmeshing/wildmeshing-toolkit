#pragma once

#include <map>
#include <memory>
#include <vector>
#include <wmtk/Mesh.hpp>
#include <wmtk/MeshBase.hpp>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/attribute/TypedAttributeHandle.hpp>

namespace wmtk::submesh {

class SubMesh;

/**
 * The embedding is a wrapper for the embedding mesh for the submeshes. It contains a pointer to the
 * mesh, the attribute for the submesh tags, and a factory for SubMeshes.
 */
class Embedding : public std::enable_shared_from_this<Embedding>, public MeshBase
{
public:
    Embedding(const std::shared_ptr<Mesh>& mesh);

    std::shared_ptr<SubMesh> add_submesh();

    Mesh& mesh();
    const Mesh& mesh() const;

    attribute::TypedAttributeHandle<int64_t>& tag_handle(const PrimitiveType pt);
    attribute::Accessor<int64_t> tag_accessor(const PrimitiveType pt);

    std::vector<Tuple> get_all(PrimitiveType type) const;

    int64_t top_cell_dimension() const;

    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const;
    bool is_boundary(PrimitiveType, const Tuple& tuple) const;
    int64_t id(const simplex::Simplex& s) const;
    int64_t id(const Tuple& tuple, PrimitiveType pt) const;

private:
    std::shared_ptr<Mesh> m_mesh;

    std::map<PrimitiveType, std::string> m_tag_attribute_name;
    std::map<PrimitiveType, attribute::TypedAttributeHandle<int64_t>> m_tag_handle;

    std::vector<std::shared_ptr<SubMesh>> m_submeshes;
    int64_t m_submesh_counter = 0;
};

} // namespace wmtk::submesh