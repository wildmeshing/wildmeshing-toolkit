#pragma once

namespace wmtk::attribute {
struct AttributeManager;
class AttributeScopeHandle
{
public:
    AttributeScopeHandle(AttributeManager& manager);
    ~AttributeScopeHandle();


    void mark_failed();

private:
    AttributeManager& m_manager;
    bool m_failed = false;
};
} // namespace wmtk::attribute
