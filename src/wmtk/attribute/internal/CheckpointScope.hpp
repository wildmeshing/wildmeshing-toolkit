#pragma once
namespace wmtk::attribute {
class AttributeManager;
}
namespace wmtk::attribute::internal {
class CheckpointScope
{
public:
    CheckpointScope(wmtk::attribute::AttributeManager& manager);
    ~CheckpointScope();

private:
    wmtk::attribute::AttributeManager& m_manager;
};
} // namespace wmtk::attribute::internal
