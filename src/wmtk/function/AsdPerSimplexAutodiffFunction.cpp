#include "AsdPerSimplexAutodiffFunction.hpp"
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/SimplexGetter.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/internal/SimplexEqualFunctor.hpp>


namespace wmtk::function {

AsdPerSimplexAutodiffFunction::AsdPerSimplexAutodiffFunction(
    const Mesh& mesh,
    const PrimitiveType& domain_simplex_type,
    const MeshAttributeHandle<double>& variable_attribute_handle)
    : m_coordinate_attribute_handle(variable_attribute_handle)
    , m_mesh(mesh)
{}

AsdPerSimplexAutodiffFunction::~AsdPerSimplexAutodiffFunction() = default;

std::vector<AsdPerSimplexAutodiffFunction::DSVec> AsdPerSimplexAutodiffFunction::get_coordinates(
    const Simplex& domain_simplex,
    const std::optional<simplex::Simplex>& variable_simplex_opt) const
{
    ConstAccessor<double> pos = mesh().create_const_accessor(get_coordinate_attribute_handle());
    return get_coordinates(pos, domain_simplex, variable_simplex_opt);
}

std::vector<AsdPerSimplexAutodiffFunction::DSVec> AsdPerSimplexAutodiffFunction::get_coordinates(
    const ConstAccessor<double>& accessor,
    const Simplex& domain_simplex,
    const std::optional<simplex::Simplex>& variable_simplex_opt) const
{
    auto [attrs, index] = utils::get_simplex_vertex_attributes(
        mesh(),
        accessor,
        domain_simplex,
        variable_simplex_opt->tuple());

    std::vector<DSVec> ret;
    ret.reserve(attrs.size());

    for (size_t i = 0; i < ret.size(); ++i) {
        ret.emplace_back(
            i == index ? utils::as_DScalar<DScalar>(attrs[i]) : attrs[i].cast<DScalar>());
    }

    return ret;
}


double AsdPerSimplexAutodiffFunction::get_value(const simplex::Simplex& domain_simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());

    // return the energy
    return eval(domain_simplex, get_coordinates(domain_simplex)).getValue();
}

Eigen::VectorXd AsdPerSimplexAutodiffFunction::get_gradient(
    const simplex::Simplex& domain_simplex,
    const simplex::Simplex& variable_simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());

    // return the gradient
    return eval(domain_simplex, get_coordinates(domain_simplex, variable_simplex)).getGradient();
}

Eigen::MatrixXd AsdPerSimplexAutodiffFunction::get_hessian(
    const simplex::Simplex& domain_simplex,
    const simplex::Simplex& variable_simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());

    // return the hessian
    return eval(domain_simplex, get_coordinates(domain_simplex, variable_simplex)).getHessian();
}

} // namespace wmtk::function
