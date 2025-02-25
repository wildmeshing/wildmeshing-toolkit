#include "rgb.hpp"
namespace wmtk {
class TriMesh;
}
namespace wmtk::components::rgb {
namespace operations {
class RGBEdgeSplit;
class RGBTriEdgeSwap;
} // namespace operations
class RGB2 : public RGB
{
public:
    using RGB::RGB;
    RGB2(const RGBOptions& opts);
    virtual ~RGB2() override;
    void run() final;

private:
    void register_attributes();

    void create_split();
    void create_swap();

private:
    TriMesh& mesh();
    const TriMesh& mesh() const;

    std::shared_ptr<operations::RGBEdgeSplit> m_split_op;
    std::shared_ptr<operations::RGBTriEdgeSwap> m_swap_op;

    attribute::MeshAttributeHandle m_todo_handle;
    attribute::MeshAttributeHandle m_edge_length_handle;
    attribute::MeshAttributeHandle m_triangle_level_handle;
    attribute::MeshAttributeHandle m_edge_level_handle;
};
} // namespace wmtk::components::rgb
