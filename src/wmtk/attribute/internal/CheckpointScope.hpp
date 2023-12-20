#pragma once
namespace wmtk::attribute {
class AttributeManager;
}
namespace wmtk::attribute::internal {
class CheckpointScope
{
public:
    CheckpointScope(const AttributeManager& manager);
    ~CheckpointScope();

private:
    const AttributeManager& m_manager;
};
} // namespace wmtk::attribute::internal
