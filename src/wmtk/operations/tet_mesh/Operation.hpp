#pragma once
#include "../Operation.hpp"

namespace wmtk {
class TetMesh;

namespace operations::tet_mesh {

class Operation : public wmtk::operations::Operation
{
public:
    Operation(Mesh& m);

protected:
    TetMesh& mesh() const;
};


} // namespace operations::tet_mesh
} // namespace wmtk
