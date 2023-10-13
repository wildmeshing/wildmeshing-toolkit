#include "Function.hpp"
namespace wmtk::function {

Function::Function(const Mesh& mesh)
    : m_mesh(mesh)
{}

Function::~Function() = default;


const Mesh& Function::mesh() const
{
    return m_mesh;
}
} // namespace wmtk::function
