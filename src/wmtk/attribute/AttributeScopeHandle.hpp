#pragma once

namespace wmtk::attribute {
struct AttributeManager;
class AttributeScopeHandle
{
public:
    AttributeScopeHandle(AttributeManager& manager);
    AttributeScopeHandle(AttributeScopeHandle&&);
    ~AttributeScopeHandle();


    void mark_failed();

private:
    AttributeManager& m_manager;
    bool m_failed = false;
    bool m_was_moved = false;
};
} // namespace wmtk::attribute
