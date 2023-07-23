#pragma once
#include <wmtk/Primitive.hpp>
namespace wmtk {
    class Mesh;
}
namespace wmtk::utils {
    long active_simplices(const Mesh& mesh, PrimitiveType type);
}
