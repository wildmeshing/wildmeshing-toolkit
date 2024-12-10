
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include "internal/IndexSimplexMapper.hpp"
namespace wmtk::utils {
namespace {

template <int Dim>
std::array<int64_t, Dim> indices(const Mesh& m, const simplex::Simplex& s)
{
    assert(Dim == get_primitive_type_id(s.primitive_type()) + 1);
    auto cof = simplex::cofaces_single_dimension(m, s, PrimitiveType::Vertex);
}
} // namespace

bool verify_simplex_index_valences(const Mesh& m)
{
    internal::IndexSimplexMapper mapper(m);


    return true;
}
} // namespace wmtk::utils
