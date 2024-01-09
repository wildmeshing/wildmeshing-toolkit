#pragma once
#include <set>
#include <wmtk/Mesh.hpp>

namespace wmtk::components::multimesh::utils {
// An edge on a seam of a uv mesh is a seam edge. Every seam edge on the uv mesh has a corresponding
// edge also on the uv mesh where both edges are mapped to the same edge on the position mesh. We
// call the two edges in uv mesh sibling edges.
// To avoid a seam edge being mapped to reversely oriented sibling seam edge the sufficient
// condition is to have critical points that are:
//  1. end of seams
//  2. T junction points.
//
//  We need these definitions :
// - Let p be a vertex on a position mesh.
// - Let V map from one p to a collection V_p = {v_i} where each v_i is a vertex on the uv_mesh.
// - Let BV be the set of boundary vertices of the uv mesh,
// - Let BP be the set of boundary vertices of the position mesh
// - Let P be a map that acts as an “inverse” to V()
//     - P(v_i) gives you a p such that V(p) contains v_i.
//     - e.g. V(a) = (b,c), then P(b)=a and P(c)=a.
// The algorithm to find critical points is:
// For each v in BV, v is a critical point if
//  1. p is in BP, |V_p|  != 1, OR
//  2. p in not BP, |V_p| != 2

// input: pointer to uv mesh, pointer to position mesh
// output: std::set<Tuple> the set of Tuple whose corresponding vertices are critical points.
std::set<Tuple> find_critical_points(const Mesh& uv_mesh, const Mesh& position_mesh);

} // namespace wmtk::components::multimesh::utils