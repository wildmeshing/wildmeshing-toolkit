#pragma once
#include <memory>
#include <vector>
namespace wmtk {
    class Mesh;
}
std::shared_ptr<wmtk::Mesh> make_mesh();
auto make_child(wmtk::Mesh& m, const std::vector<int64_t>& path)
    -> std::vector<std::shared_ptr<wmtk::Mesh>>;
