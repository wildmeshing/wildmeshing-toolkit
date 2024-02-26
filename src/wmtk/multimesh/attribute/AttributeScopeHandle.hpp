#pragma once
#include <vector>

namespace wmtk {
class Mesh;
}
namespace wmtk::attribute {
class AttributeScopeHandle;
}
namespace wmtk::multimesh::attribute {
class AttributeScopeHandle
{
public:
    using single_handle_type = wmtk::attribute::AttributeScopeHandle;
    AttributeScopeHandle(Mesh& m);
    ~AttributeScopeHandle() = default;

    void mark_failed();

private:
    std::vector<single_handle_type> m_scopes;
};
} // namespace wmtk::multimesh::attribute
