#include "Function.hpp"
#include <wmtk/simplex/top_level_cofaces.hpp>
namespace wmtk::function {

Function::Function(const Mesh& mesh)
    : m_mesh(mesh)
{}

Function::~Function() = default;

double Function::get_one_ring_value(const Tuple& vertex) const
{
    auto simplices =
        simplex::top_level_cofaces_tuples(mesh(), Simplex(PrimitiveType::Vertex, vertex));
    return get_value_sum(simplices);
}
double Function::get_value_sum(const std::vector<Tuple>& top_level_simplices) const
{
    double v = 0;
    for (const Tuple& cell : top_level_simplices) {
        v += get_value(cell);
    }
    return v;
}

const Mesh& Function::mesh() const
{
    return m_mesh;
}
} // namespace wmtk::function
