#pragma once
namespace wmtk::attribute {
class AttributeManager;
}
namespace wmtk::attribute::internal {
// Scope for briefly changing the current scope of a mesh (in particular, this is used in
// parent_scope to temporarly access different meshes)
class CheckpointScope
{
public:
    CheckpointScope(const AttributeManager& manager);
    ~CheckpointScope();

private:
    const AttributeManager& m_manager;
};
} // namespace wmtk::attribute::internal
