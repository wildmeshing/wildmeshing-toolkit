#include "MeshInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/primitive_range.hpp>
namespace wmtk {
MeshInvariant::MeshInvariant(const Mesh& mesh)
    : m_mesh(mesh)
{}

MeshInvariant::~MeshInvariant() = default;
const Mesh& MeshInvariant::mesh() const
{
    return m_mesh;
}

bool MeshInvariant::directly_modified_after(PrimitiveType type, const std::vector<Tuple>& ts) const
{
    for (const Tuple& t : ts) {
        const Simplex s = Simplex(type, t);
        for (const PrimitiveType pt : wmtk::utils::primitive_below(mesh().top_simplex_type())) {
            std::vector<Tuple> modified_tuples =
                //
                {};
            //    simplex::bounded_simplices_single_dimension_tuples(mesh(), pt, s);

            if (!after(pt, modified_tuples)) {
                return false;
            }
        }
    }
    return true;
}

} // namespace wmtk
