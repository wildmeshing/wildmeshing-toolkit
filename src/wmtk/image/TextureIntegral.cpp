#pragma once

#include "TextureIntegral.h"

namespace wmtk {

TextureIntegral::TextureIntegral(std::array<wmtk::Image, 3> data)
    : m_data(std::move(data))
{}

void TextureIntegral::get_error_per_triangle(
    lagrange::span<const std::array<double, 6>> input_triangles,
    lagrange::span<double> output_errors)
{}

void TextureIntegral::get_integral_per_triangle(
    lagrange::span<const std::array<double, 6>> input_triangles,
    lagrange::span<std::array<double, 3>> output_integrals)
{}

} // namespace wmtk
