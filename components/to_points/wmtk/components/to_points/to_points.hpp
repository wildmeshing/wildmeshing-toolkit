#pragma once


#include <string>
#include <memory>

namespace wmtk {
    class Mesh;
    class PointMesh;
    namespace attribute {
        class MeshAttributeHandle;
    }
}

namespace wmtk::components {

class ToPtsOptions
{
public:
    bool add_box;
    double box_scale;
    bool add_grid;
    double grid_spacing;
    double min_dist;
    bool remove_duplicates;
};

std::shared_ptr<PointMesh> to_points(
    const Mesh& mesh,
    const attribute::MeshAttributeHandle& pts_attr,
    const ToPtsOptions& options,
    const std::string& output_pos_attr_name = "vertices");

} // namespace wmtk::components
