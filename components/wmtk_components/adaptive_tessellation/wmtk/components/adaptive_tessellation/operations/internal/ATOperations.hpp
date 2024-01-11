#pragma once
#include <nlohmann/json.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/SumEnergy.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include "ATData.hpp"
namespace wmtk::components::operations::internal {
class ATOperations
{
public:
    ATData& m_atdata;
    std::vector<std::shared_ptr<wmtk::operations::Operation>> m_ops;
    double m_target_edge_length;

    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_edge_length_update;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_3d_position_update;
    Accessor<double> m_edge_length_accessor;
    std::function<std::vector<double>(const Simplex&)> m_long_edges_first;
    std::function<std::vector<double>(const Simplex&)> m_short_edges_first;

    wmtk::components::function::utils::ThreeChannelPositionMapEvaluator m_evaluator;
    std::shared_ptr<wmtk::function::PerSimplexFunction> m_accuracy_energy;
    std::shared_ptr<wmtk::function::TriangleAMIPS> m_amips_energy;
    std::shared_ptr<wmtk::function::PositionMapAMIPS> m_3d_amips_energy;
    std::shared_ptr<wmtk::function::SumEnergy> m_sum_energy;

public:
    // constructor
    ATOperations(ATData& atdata, double target_edge_length);
    void set_energies();
    void AT_split_single_edge_mesh(Mesh* edge_meshi_ptr);
    void AT_smooth_interior();
    void AT_smooth_interior(std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr);
    void AT_split_interior();
    void AT_split_boundary();
    void AT_collapse_interior(std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr);
    void AT_swap_interior(std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr);


    void at_operation(const nlohmann::json& j);
};
} // namespace wmtk::components::operations::internal