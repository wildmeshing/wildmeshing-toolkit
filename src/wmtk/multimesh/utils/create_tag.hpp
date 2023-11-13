#pragma once
#include <set>
#include <vector>
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::multimesh::utils {

bool is_root(std::vector<int>& parents, int vid);

std::vector<int> create_parents(std::vector<Tuple>& tuples);

void set_root(std::vector<int>& parents, int vid, int root);
int get_root(std::vector<int>& parents, int vid);

void set_union(std::vector<int>& parents, int vid1, int vid2);

void union_find(std::set<int>& critical_points, std::vector<int>& parents, int vid1, int vid2);
} // namespace wmtk::multimesh::utils