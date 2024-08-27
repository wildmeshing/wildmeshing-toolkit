#pragma once

#include <memory>
#include <wmtk/PrimitiveType.hpp>

namespace wmtk {
class Mesh;
}
namespace wmtk::tests::tools {


std::shared_ptr<Mesh> single_simplex_mesh(const wmtk::PrimitiveType pt);
}
