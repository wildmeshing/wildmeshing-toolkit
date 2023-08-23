#pragma once
#include "../Operation.hpp"

namespace wmtk {
class TriMesh;

namespace operations::tri_mesh {

class Operation : public wmtk::operations::Operation
{
public:
    Operation(Mesh& m);

protected:
    TriMesh& mesh() const;
};


} // namespace operations::tri_mesh
} // namespace wmtk
