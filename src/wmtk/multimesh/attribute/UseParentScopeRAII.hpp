#pragma once

namespace wmtk {

class Mesh;
namespace multimesh::attribute {
class UseParentScopeRAII
{
public:
    UseParentScopeRAII(Mesh& m);
    ~UseParentScopeRAII();

private:
    Mesh& m_mesh;
};
} // namespace multimesh::attribute
} // namespace wmtk
