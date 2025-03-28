#include "DEBUG_Mesh.hpp"
#include <catch2/catch_test_macros.hpp>

namespace wmtk::tests {


bool DEBUG_Mesh::operator==(const DEBUG_Mesh& o) const
{
    return static_cast<const Mesh&>(*this) == static_cast<const Mesh&>(o);
}
bool DEBUG_Mesh::operator!=(const DEBUG_Mesh& o) const
{
    return !(*this == o);
}


void DEBUG_Mesh::print_state() const {}


void DEBUG_Mesh::reserve_attributes(PrimitiveType type, int64_t size)
{
    Mesh::reserve_attributes(type, size);
}


} // namespace wmtk::tests
