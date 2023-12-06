#pragma once

namespace wmtk::attribute {
struct AttributeManager;
class Mesh;
class AttributeScopeHandle
{
public:
    AttributeScopeHandle(Mesh& mesh);
    AttributeScopeHandle(AttributeManager& manager);
    ~AttributeScopeHandle();


    void mark_failed();

private:
    AttributeManager& m_manager;
    bool m_failed = false;
};
} // namespace wmtk::attribute
