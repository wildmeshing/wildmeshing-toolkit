#pragma once
#include <array>
#include <optional>
#include <vector>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/autogen/Dart.hpp>

namespace wmtk::tests::tools {

std::vector<int8_t> darts_using_faces(
    PrimitiveType mesh_type,
    const std::array<std::optional<int64_t>, 3>& indices);
}
