#pragma once
#include <map>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
std::map<int64_t, std::vector<wmtk::Tuple>> compress_indices(
    const wmtk::attribute::MeshAttributeHandle& h);

