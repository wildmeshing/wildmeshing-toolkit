#pragma once

#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>

#include <wmtk/Types.hpp>
std::pair<wmtk::MatrixXl, std::vector<wmtk::Tuple>> compress_indices(
    const wmtk::TriMesh& m,
    const wmtk::PrimitiveType& pt,
    const std::vector<wmtk::Tuple>& tups);
