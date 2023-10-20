#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>
#include "DifferentiablePerSimplexFunction.hpp"
#include "Function.hpp"
namespace wmtk {
namespace function {
class LocallyDifferentiableFunction : public Function
{
public:
    LocallyDifferentiableFunction(
        const Mesh& mesh,
        const std::unique_ptr<DifferentiablePerSimplexFunction>& function);
    virtual ~LocallyDifferentiableFunction();

public:
    Eigen::VectorXd get_one_ring_gradient(
        const Simplex& my_simplex,
        const PrimitiveType& cofaces_type) const;
    Eigen::MatrixXd get_one_ring_hessian(
        const Simplex& my_simplex,
        const PrimitiveType& cofaces_type) const;

    double get_value_sum(const std::vector<Simplex>& coface_simplices) const;
    Eigen::VectorXd get_gradient_sum(const std::vector<Simplex>& coface_simplices) const;
    Eigen::MatrixXd get_hessian_sum(const std::vector<Simplex>& coface_simplices) const;

    /**
     * @brief Get the one ring simplices centered at vertex object.
     *  m_function wrt to vertex assums the tuple is the vertex that's differentiated wrt
     *(thus can  not use co-face, since the returned value of co-face are in arbitrary order)
     *
     * @param vertex_tuple
     * @return std::vector<Simplex>& the simplices whose tuple is centered at the vertex and is
     * defined over the same type as m_function
     */
    std::vector<Simplex>& get_one_ring_simplices_centered_at_vertex(
        const Tuple& vertex_tuple) const;
    long embedded_dimension() const;

private:
    const std::unique_ptr<DifferentiablePerSimplexFunction>& m_function;
};
} // namespace function
} // namespace wmtk