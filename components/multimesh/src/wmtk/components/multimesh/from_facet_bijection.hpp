#pragma once

namespace wmtk {
class Mesh;
} // namespace wmtk
namespace wmtk::components::multimesh {


void from_facet_bijection(Mesh& parent, Mesh& child);

}
