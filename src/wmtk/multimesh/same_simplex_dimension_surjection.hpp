#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Mesh;
}

namespace wmtk::multimesh {

// Handles the "trivial " mapping case as we see in OBJ files
// parent and child are assumed to be homogeneous meshes of the same dimension and the same number
// of top level simplices It is assumed that for each index between [0,num_top_level_simplices) it
// is assumed that the tuple parent.tuple_from_id(pt,index) should be mapped to
// child.tuple_from_id(pt,index)
std::vector<std::array<Tuple, 2>> same_simplex_dimension_surjection(
    const Mesh& parent,
    const Mesh& child);
// same as above except  the mapping selects a subset of parent_simplices
// the tuple parent.tuple_from_id(pt,parent_simplices[index]) should be mapped to
// child.tuple_from_id(pt,index)
std::vector<std::array<Tuple, 2>> same_simplex_dimension_surjection(
    const Mesh& parent,
    const Mesh& child,
    const std::vector<long>& parent_simplices);
} // namespace wmtk::multimesh
