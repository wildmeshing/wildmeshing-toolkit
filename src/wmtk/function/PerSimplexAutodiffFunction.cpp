#include "PerSimplexAutodiffFunction.hpp"
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/SimplexGetter.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/internal/SimplexEqualFunctor.hpp>


namespace wmtk::function {

PerSimplexAutodiffFunction::PerSimplexAutodiffFunction(
    const Mesh& mesh,
    const PrimitiveType primitive_type,
    const MeshAttributeHandle<double>& variable_attribute_handle)
    : PerSimplexFunction(mesh, primitive_type, variable_attribute_handle)
{}

PerSimplexAutodiffFunction::~PerSimplexAutodiffFunction() = default;

std::vector<PerSimplexAutodiffFunction::DSVec> PerSimplexAutodiffFunction::get_coordinates(
    const simplex::Simplex& domain_simplex,
    const std::optional<simplex::Simplex>& variable_simplex_opt) const
{
    ConstAccessor<double> pos = mesh().create_const_accessor(attribute_handle());
    return get_coordinates(pos, domain_simplex, variable_simplex_opt);
}

std::vector<PerSimplexAutodiffFunction::DSVec> PerSimplexAutodiffFunction::get_coordinates(
    const ConstAccessor<double>& accessor,
    const simplex::Simplex& domain_simplex,
    const std::optional<simplex::Simplex>& variable_simplex_opt) const
{
    auto [attrs, index] = utils::get_simplex_attributes(
        mesh(),
        accessor,
        m_primitive_type,
        domain_simplex,
        variable_simplex_opt.has_value() ? variable_simplex_opt->tuple() : std::optional<Tuple>());

    std::vector<DSVec> ret;
    ret.reserve(attrs.size());

    for (size_t i = 0; i < attrs.size(); ++i) {
        ret.emplace_back(
            i == index ? utils::as_DScalar<DScalar>(attrs[i]) : attrs[i].cast<DScalar>());
    }

    return ret;
}


double PerSimplexAutodiffFunction::get_value(const simplex::Simplex& domain_simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());

    // return the energy
    return eval(domain_simplex, get_coordinates(domain_simplex)).getValue();
}

Eigen::VectorXd PerSimplexAutodiffFunction::get_gradient(
    const simplex::Simplex& domain_simplex,
    const simplex::Simplex& variable_simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());

    // return the gradient
    return eval(domain_simplex, get_coordinates(domain_simplex, variable_simplex)).getGradient();
}

Eigen::MatrixXd PerSimplexAutodiffFunction::get_hessian(
    const simplex::Simplex& domain_simplex,
    const simplex::Simplex& variable_simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());

    // return the hessian
    return eval(domain_simplex, get_coordinates(domain_simplex, variable_simplex)).getHessian();
}

} // namespace wmtk::function
