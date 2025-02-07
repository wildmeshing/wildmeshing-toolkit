#pragma once

#include <functional>
#include <map>
#include <memory>
#include <vector>
#include <wmtk/Mesh.hpp>
#include <wmtk/MeshBase.hpp>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>

namespace wmtk::operations {
class EdgeSplit;
class EdgeCollapse;
} // namespace wmtk::operations

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
    const attribute::Accessor<int64_t> tag_accessor(const PrimitiveType pt) const;

    std::vector<Tuple> get_all(PrimitiveType type) const override;
    std::vector<simplex::IdSimplex> get_all_id_simplex(PrimitiveType type) const override;

    int64_t top_cell_dimension() const;

    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const;
    bool is_boundary(PrimitiveType, const Tuple& tuple) const;
    int64_t id(const simplex::Simplex& s) const;
    int64_t id(const Tuple& tuple, PrimitiveType pt) const;

    bool has_child_mesh() const;

    bool simplex_is_in_submesh(const simplex::Simplex& s) const;

    void set_split_strategies(operations::EdgeSplit& split) const;

    void set_collapse_strategies(operations::EdgeCollapse& collapse) const;

    std::function<bool(const simplex::Simplex&)> substructure_predicate() const;

private:
    std::shared_ptr<Mesh> m_mesh;

    std::map<PrimitiveType, std::string> m_tag_attribute_name;
    std::map<PrimitiveType, attribute::TypedAttributeHandle<int64_t>> m_tag_handle;

    std::vector<std::shared_ptr<SubMesh>> m_submeshes;

    std::map<PrimitiveType, std::shared_ptr<operations::SplitNewAttributeStrategy<int64_t>>>
        m_split_new;
    std::map<PrimitiveType, std::shared_ptr<operations::CollapseNewAttributeStrategy<int64_t>>>
        m_collapse_new;

    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<int64_t, int64_t>> m_transfer;

    std::function<bool(const simplex::Simplex&)> m_substructure_predicate;
};

} // namespace wmtk::submesh