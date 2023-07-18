#pragma once

namespace wmtk {
class AttributeManager;
class AttributeScopeHandle
{
public:
    AttributeScopeHandle(AttributeManager& manager);
    ~AttributeScopeHandle();

private:
    AttributeManager& m_manager;
};
} // namespace wmtk
