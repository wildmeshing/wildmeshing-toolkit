#pragma once
#include <wmtk/function/utils/autodiff.h>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>
// #include <wmtk/function/utils/AutoDiffTypes.hpp>
#include "Function.hpp"
namespace wmtk {
namespace function {

class DifferentiableFunction : public Function
{
public:
    DifferentiableFunction(
        const Mesh& mesh,
        const MeshAttributeHandle<double>& vertex_attribute_handle);

    virtual ~DifferentiableFunction() = default;

    const MeshAttributeHandle<double> m_vertex_attribute_handle;

    virtual Eigen::VectorXd get_gradient(const Tuple& tuple) const = 0;
    virtual Eigen::MatrixXd get_hessian(const Tuple& tuple) const = 0;
};
} // namespace function
} // namespace wmtk
