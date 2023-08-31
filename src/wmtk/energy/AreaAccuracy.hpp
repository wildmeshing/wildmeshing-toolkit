#include "DifferentiableEnergy.hpp"
namespace wmtk {
namespace energy {

class AreaAccuracy : public DifferentiableEnergy
{
public:
    AreaAccuracy(const TriMesh& mesh1, const TriMesh& mesh2);

    double energy_eval(const Tuple& tuple) const override;
    DScalar energy_eval_autodiff(const Tuple& tuple) const override;

    /**
     * @brief gradient defined wrt the first vertex
     *
     * @param uv1
     * @param uv2
     * @param uv3
     * @return can be double or DScalar
     */
    template <typename T>
    static T
    energy_eval(const Eigen::Vector2d& uv1, const Eigen::Vector2d& uv2, const Eigen::Vector2d& uv3);

protected:
    const TriMesh& m_position_mesh;
    const MeshAttributeHandle<double> m_3d_position_handle;
};
} // namespace energy
} // namespace wmtk