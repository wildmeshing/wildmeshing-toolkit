
#include "single_simplex_mesh.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
namespace wmtk {
class Mesh;
}
namespace wmtk::tests::tools {


std::shared_ptr<Mesh> single_simplex_mesh(const wmtk::PrimitiveType pt)
{
    RowVectorX<int64_t> data = RowVectorX<int64_t>::LinSpaced(5, 0, 4);
    switch (pt) {
    case PrimitiveType::Vertex: return std::make_shared<wmtk::PointMesh>(1);
    case PrimitiveType::Edge: {
        auto e = std::make_shared<wmtk::EdgeMesh>();
        e->initialize(data.leftCols<2>());
        return e;
    }
    case PrimitiveType::Triangle: {
        auto e = std::make_shared<wmtk::TriMesh>();
        e->initialize(data.leftCols<3>());
        return e;
    }
    case PrimitiveType::Tetrahedron: {
        auto e = std::make_shared<wmtk::TetMesh>();
        e->initialize(data.leftCols<4>());
        return e;
    }
    }
    assert(false);
    return {};
}
} // namespace wmtk::tests::tools
