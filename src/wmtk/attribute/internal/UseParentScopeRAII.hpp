#pragma once

namespace wmtk {

class Mesh;
namespace attribute {
class UseParentScopeRAII
{
public:
    UseParentScopeRAII(Mesh& m);
    ~UseParentScopeRAII();

private:
    Mesh& m_mesh;
};
} // namespace attribute
} // namespace wmtk
