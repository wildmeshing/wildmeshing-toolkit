#pragma once
#include <optional>
#include <vector>
#include <wmtk/autogen/Dart.hpp>
#include <wmtk/PrimitiveType.hpp>
#include <array>

namespace wmtk::tests::tools {

    std::vector<autogen::Dart> darts_using_faces(PrimitiveType mesh_type, 
            const std::array<std::optional<int64_t>,3>& indices);
}
