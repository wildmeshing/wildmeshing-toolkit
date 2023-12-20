#pragma once
namespace wmtk::attribute {
class AttributeManager;
}
namespace wmtk::attribute::internal {
class CheckpointScope
{
public:
    CheckpointScope(const wmtk::attribute::AttributeManager& manager);
    ~CheckpointScope();

private:
    const wmtk::attribute::AttributeManager& m_manager;
};
} // namespace wmtk::attribute::internal
