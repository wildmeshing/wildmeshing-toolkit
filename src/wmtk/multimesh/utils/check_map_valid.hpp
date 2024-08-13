#pragma once

namespace wmtk {
class Mesh;
namespace multimesh {
class MultiMeshManager;
}
} // namespace wmtk

namespace wmtk::multimesh::utils {
bool check_child_maps_valid(const Mesh& m);
bool check_parent_map_valid(const Mesh& m);

} // namespace wmtk::multimesh::utils
