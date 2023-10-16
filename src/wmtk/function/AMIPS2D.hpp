#pragma once
#include "AMIPS.hpp"
namespace wmtk::function {
class AMIPS2D : public AMIPS
{
public:
    AMIPS2D(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle);

protected:
    DScalar get_value_autodiff(const Tuple& tuple) const override;

    /**
     * @brief gradient defined wrt the first vertex
     *
     * @param uv0
     * @param uv1
     * @param uv2
     * @return can be double or DScalar
     */
    template <typename T>
    T function_eval(
        const Eigen::Matrix<T, 2, 1>& uv0,
        const Eigen::Vector2d& uv1,
        const Eigen::Vector2d& uv2) const;
    template <typename T>
    T function_eval(const Tuple& tuple) const;

private:
    static Eigen::Matrix<double, 3, 2> get_target_triangle(double scaling);
};
