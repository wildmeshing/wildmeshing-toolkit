#pragma once

#include <cstddef>
#include <array>
#include <vector>
#include <cstdlib>
#include <random>
#include <map>
#include <Eigen/Dense>


namespace wmtk {
using Point3D = std::array<double, 3>;
using Tetrahedron = std::array<size_t, 4>;
using Point2D = std::array<double, 2>;
using Triangle = std::array<size_t, 3>;

} // namespace wmtk