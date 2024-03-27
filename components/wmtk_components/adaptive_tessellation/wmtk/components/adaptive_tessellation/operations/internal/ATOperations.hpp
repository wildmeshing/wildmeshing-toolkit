#pragma once
#include <nlohmann/json.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/DistanceEnergy.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/DistanceEnergyNonDiff.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/SumEnergy.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/operations/Operation.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include "ATData.hpp"
using namespace wmtk;
namespace wmtk::components::operations::internal {
class ATOperations
{
public:
    ATData& m_atdata;
    double m_target_distance;
    double m_target_edge_length;
    double m_envelope_size;
    double m_barrier_weight;
    double m_barrier_triangle_area;
    double m_distance_weight;
    double m_amips_weight;
    bool m_area_weighted_amips;

    wmtk::attribute::Accessor<double> m_uv_accessor;
    wmtk::attribute::Accessor<double> m_uvmesh_xyz_accessor;
    wmtk::attribute::Accessor<double> m_distance_error_accessor;
    wmtk::attribute::Accessor<double> m_amips_error_accessor;
    wmtk::attribute::Accessor<double> m_3d_edge_length_accessor;
    wmtk::attribute::Accessor<double> m_curved_edge_length_accessor;

    wmtk::attribute::Accessor<int64_t> m_face_rgb_state_accessor;
    wmtk::attribute::Accessor<int64_t> m_edge_rgb_state_accessor;
    wmtk::attribute::Accessor<int64_t> m_edge_todo_accessor;

    std::vector<std::shared_ptr<wmtk::operations::Operation>> m_ops;
    std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
        m_evaluator_ptr;
    std::shared_ptr<wmtk::components::function::utils::IntegralBase> m_integral_ptr;

    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_3d_edge_length_update;

    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_uvmesh_xyz_update;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_distance_error_update;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_amips_error_update;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_curved_edge_length_update;

    std::function<std::vector<double>(const Simplex&)> m_high_error_edges_first;
    std::function<std::vector<double>(const Simplex&)> m_high_distance_edges_first;
    std::function<std::vector<double>(const Simplex&)> m_high_distance_faces_first;
    std::function<std::vector<double>(const Simplex&)> m_triangle_distance_edge_length;
    std::function<std::vector<double>(const Simplex&)> m_high_amips_edges_first;
    std::function<std::vector<double>(const Simplex&)> m_edge_length_weighted_distance_priority;
    std::function<std::vector<double>(const Simplex&)> m_long_edges_first;
    std::function<std::vector<double>(const Simplex&)> m_short_edges_first;
    std::function<std::vector<long>(const Simplex&)> m_valence_improvement;

    std::shared_ptr<wmtk::function::DistanceEnergy> m_distance_energy;
    std::shared_ptr<wmtk::function::DistanceEnergyNonDiff> m_distance_nondiff_energy;

    std::shared_ptr<wmtk::function::AMIPS> m_2d_amips_energy;
    std::shared_ptr<wmtk::function::PositionMapAMIPS> m_3d_amips_energy;

public:
    // constructor
    ATOperations(
        ATData& atdata,
        double target_distance,
        double target_edge_length,
        double envelope_size,
        double barrier_weight,
        double barrier_triangle_area,
        double distance_weight,
        double amips_weight,
        bool area_weighted_amips);
    void set_energies();
    double amips3d_in_double(
        Eigen::Vector2<double>& uv0,
        Eigen::Vector2<double>& uv1,
        Eigen::Vector2<double>& uv2);
    void AT_smooth_interior(std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr);
    void AT_edge_split(std::function<std::vector<double>(const Simplex&)>& priority);
    void AT_3d_edge_split(std::function<std::vector<double>(const Simplex&)>& priority);
    void AT_split_boundary();
    void AT_swap_interior(
        std::function<std::vector<double>(const Simplex&)>& priority,
        std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr);
    void AT_rg_refine(std::function<std::vector<double>(const Simplex&)>& priority);

    int64_t AT_rgb_split();
    int64_t AT_rgb_swap();
    ///// update
    void set_uvmesh_xyz_update_rule_initialize();

    void set_3d_edge_length_update_rule();
    void initialize_3d_edge_length();
    void set_distance_error_update_rule();
    void initialize_distance_error();
    void set_3d_amips_error_update_rule();
    void initialize_3d_amips_error();
    void set_curved_edge_length_update_rule();
    void initialize_curved_edge_length();

    static double curved_edge_length_on_displaced_surface(
        const Eigen::Vector2d& uv0,
        const Eigen::Vector2d& uv1,
        const std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
            m_evaluator_ptr);
};
} // namespace wmtk::components::operations::internal