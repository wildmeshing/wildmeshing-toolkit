#include <wmtk/image/Image.hpp>
#include <wmtk/image/Sampling.hpp>
#include "AutodiffFunction.hpp"
namespace wmtk {
namespace function {

class AreaAccuracy : public AutodiffFunction
{
public:
    AreaAccuracy(const TriMesh& mesh1, const TriMesh& mesh2);

    DScalar get_value_autodiff(const Tuple& tuple) const override;

    /**
     * @brief gradient defined wrt the first vertex
     *
     * @param uv1
     * @param uv2
     * @param uv3
     * @return can be double or DScalar
     */
    template <typename T>
    static T function_eval(
        const Eigen::Vector2d& uv1,
        const Eigen::Vector2d& uv2,
        const Eigen::Vector2d& uv3);

protected:
    const TriMesh& m_position_mesh;
    const MeshAttributeHandle<double> m_3d_position_handle;
};
} // namespace function
} // namespace wmtk