#pragma once
#include <Eigen/Core>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/function/DifferentiableFunction.hpp>
#include <wmtk/function/Function.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/utils/FunctionInterface.hpp>
namespace wmtk {
class Optimization
{
public:
    Optimization() = default;
    ~Optimization();

    Optimization(
        FunctionInterface& function_interface,
        const TriMesh& m,
        const InvariantCollection& invariants,
        const bool second_order,
        const bool line_search);

    Optimization(
        const Tuple& tuple,
        Accessor<double>& accessor,
        const function::DifferentiableFunction& function,
        const TriMesh& m,
        const InvariantCollection& invariants,
        const bool second_order,
        const bool line_search);


    FunctionInterface m_function_interface;
    TriMesh m_mesh;
    const InvariantCollection& m_invariants;
    const bool m_second_order;
    const bool m_line_search;


    void optimize2d(const Eigen::Vector2d& current_pos);
};
} // namespace wmtk