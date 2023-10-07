#pragma once
#include <memory>

namespace wmtk {
class Mesh;
class Tuple;

class InvariantCollection;
namespace operations::utils {


// Initializes any invariants for splitting (which is None by default, but enabling a pattern
// with other operations)
std::shared_ptr<InvariantCollection> multimesh_edge_split_invariants(const Mesh& m);

void multi_mesh_edge_split(Mesh& mesh, const Tuple& t);


} // namespace operations::utils
} // namespace wmtk
