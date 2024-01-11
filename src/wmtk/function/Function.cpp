#include "Function.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::function {

Mesh& Function::mesh()
{
    return const_cast<Mesh&>(const_cast<const Function*>(this)->mesh());
}
Function::Function() = default;


} // namespace wmtk::function
