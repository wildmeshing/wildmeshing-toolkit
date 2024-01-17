#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
namespace wmtk::simplex {

namespace {
template <typename MeshType> // use a DEBUG mesh type
void check_match_below_simplex_type(const MeshType& mesh, const Simplex& a, const Simplex& b)
{
    PrimitiveType min_type = std::min(a.primitive_type(), b.primitive_type());

    for (int i = 0; i <= get_primitive_type_id(min_type); ++i) {
        PrimitiveType cur_primitive_type = static_cast<PrimitiveType>(i);
        CHECK(mesh.id(a.tuple(), cur_primitive_type) == mesh.id(b.tuple(), cur_primitive_type));
    }
}
} // namespace
} // namespace wmtk::simplex
