#pragma once

#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/ConstAccessor.hpp>

namespace wmtk::function::utils {

template <typename T>
std::tuple<std::vector<Eigen::Matrix<T, Eigen::Dynamic, 1>>, size_t> get_simplex_vertex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::ConstAccessor<T>& accessor,
    const attribute::MeshAttributeHandle<T>& variable_attribute_handle,
    const wmtk::Simplex& simplex,
    const std::optional<wmtk::Tuple>& vertex_marker = {});
}