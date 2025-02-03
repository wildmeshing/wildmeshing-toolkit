#pragma once

#include <map>
#include <vector>
#include <wmtk/Mesh.hpp>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/attribute/TypedAttributeHandle.hpp>

namespace wmtk::submesh {

class SubMesh;

/**
 * The embedding is a wrapper for the embedding mesh for the submeshes. It contains a pointer to the
 * mesh, the attribute for the submesh tags, and a factory for SubMeshes.
 */
class Embedding
{
public:
    Embedding(const std::shared_ptr<Mesh>& mesh);

    std::shared_ptr<SubMesh> add_submesh();

    Mesh& mesh();
    const Mesh& mesh() const;

    attribute::TypedAttributeHandle<int64_t>& tag_handle(const PrimitiveType pt);
    attribute::Accessor<int64_t> tag_accessor(const PrimitiveType pt);

private:
    std::shared_ptr<Mesh> m_mesh;

    std::map<PrimitiveType, std::string> m_tag_attribute_name;
    std::map<PrimitiveType, attribute::TypedAttributeHandle<int64_t>> m_tag_handle;

    std::vector<std::shared_ptr<SubMesh>> m_submeshes;
    int64_t m_submesh_counter = 0;
};

} // namespace wmtk::submesh