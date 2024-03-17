#pragma once

#include <optional>
#include <wmtk/function/PerSimplexFunction.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/simplex/Simplex.hpp>

//namespace wmtk {
//    namespace simplex {
//        class Simplex;
//    }
//}
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
    AMIPS(const Mesh& mesh, const attribute::MeshAttributeHandle& attribute_handle);

    ~AMIPS(){};

public:
    double get_value(const simplex::Simplex& domain_simplex) const override;

    Eigen::VectorXd get_gradient(
        const simplex::Simplex& domain_simplex,
        const simplex::Simplex& variable_simplex) const override;
    Eigen::MatrixXd get_hessian(
        const simplex::Simplex& domain_simplex,
        const simplex::Simplex& variable_simplex) const override;

private:
    template <int64_t NV, int64_t DIM>
    std::array<double, NV * DIM> get_raw_coordinates(
        const simplex::Simplex& domain_simplex,
        const std::optional<simplex::Simplex>& variable_simplex = {}) const;
};

} // namespace wmtk::function
