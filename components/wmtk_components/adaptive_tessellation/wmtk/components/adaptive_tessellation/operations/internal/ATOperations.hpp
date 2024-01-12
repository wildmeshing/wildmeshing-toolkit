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
    double m_barrier_weight_lambda;
    double m_barrier_triangle_area;

    wmtk::components::function::utils::ThreeChannelPositionMapEvaluator m_evaluator;

    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_edge_length_update;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>> m_xyz_update;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_sum_error_update;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_quadrature_error_update;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_barrier_energy_update;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_amips_error_update;

    Accessor<double> m_uv_accessor;
    Accessor<double> m_edge_length_accessor;
    Accessor<double> m_xyz_accessor;
    Accessor<double> m_quadrature_error_accessor;
    Accessor<double> m_sum_error_accessor;
    Accessor<double> m_barrier_energy_accessor;
    Accessor<double> m_amips_error_accessor;

    std::function<std::vector<double>(const Simplex&)> m_high_error_edges_first;
    std::function<std::vector<double>(const Simplex&)> m_long_edges_first;
    std::function<std::vector<double>(const Simplex&)> m_short_edges_first;
    std::function<std::vector<double>(const Simplex&)> m_valence_improvement;

    std::shared_ptr<wmtk::function::PerSimplexFunction> m_quadrature_energy;
    std::shared_ptr<wmtk::function::TriangleAMIPS> m_amips_energy;
    std::shared_ptr<wmtk::function::PositionMapAMIPS> m_3d_amips_energy;
    std::shared_ptr<wmtk::function::SumEnergy> m_sum_energy;

public:
    // constructor
    ATOperations(
        ATData& atdata,
        double target_edge_length,
        double barrier_weight_lambda,
        double barrier_triangle_area);
    void set_energies();
    void AT_split_single_edge_mesh(Mesh* edge_meshi_ptr);
    void AT_smooth_interior();
    void AT_smooth_interior(std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr);
    void AT_split_interior(
        std::function<std::vector<double>(const Simplex&)>& priority,
        std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr);
    void AT_split_boundary();
    void AT_collapse_interior(std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr);
    void AT_swap_interior(std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr);

    ///// update
    void set_xyz_update_rule();
    void initialize_vertex_xyz();
    void set_edge_length_update_rule();
    void initialize_edge_length();
    void set_sum_error_update_rule();
    void initialize_sum_error();
    void set_quadrature_error_update_rule();
    void initialize_quadrature_error();
    void set_amips_error_update_rule();
    void initialize_amips_error();
    void set_barrier_energy_update_rule();
    void initialize_barrier_energy();

    void at_operation(const nlohmann::json& j);
};
} // namespace wmtk::components::operations::internal