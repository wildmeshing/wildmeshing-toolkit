#pragma once

#include <optional>
#include <wmtk/attribute/MeshAttributes.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>

namespace wmtk::function {

class AMIPS : public PerSimplexFunction
{
public:
    /**
     * @brief Construct a new AMIPS function
     *
     * @param mesh
     * @param attribute_handle The handle to the attribute that differentiation is with respect to
     */
    AMIPS(const Mesh& mesh, const attribute::MeshAttributeHandle<double>& attribute_handle);

    ~AMIPS(){};

public:
    double get_value(const simplex::Simplex& domain_simplex) const override;

    Eigen::VectorXd get_gradient(const Simplex& domain_simplex, const Simplex& variable_simplex)
        const override;
    Eigen::MatrixXd get_hessian(const Simplex& domain_simplex, const Simplex& variable_simplex)
        const override;

private:
    template <long NV, long DIM>
    std::array<double, NV * DIM> get_raw_coordinates(
        const Simplex& domain_simplex,
        const std::optional<Simplex>& variable_simplex = {}) const;
};
} // namespace wmtk::function
