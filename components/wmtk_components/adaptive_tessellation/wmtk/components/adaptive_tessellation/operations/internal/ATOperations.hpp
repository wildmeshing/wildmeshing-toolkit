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
    int64_t m_target_triangle_number;

    wmtk::attribute::Accessor<double> m_uv_accessor;
    wmtk::attribute::Accessor<double> m_uvmesh_xyz_accessor;
    wmtk::attribute::Accessor<double> m_distance_error_accessor;
    wmtk::attribute::Accessor<double> m_curved_edge_length_accessor;

    wmtk::attribute::Accessor<int64_t> m_face_rgb_state_accessor;
    wmtk::attribute::Accessor<int64_t> m_edge_rgb_state_accessor;
    wmtk::attribute::Accessor<int64_t> m_edge_todo_accessor;

    std::vector<std::shared_ptr<wmtk::operations::Operation>> m_ops;

    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_uvmesh_xyz_update;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_distance_error_update;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_curved_edge_length_update;

    std::function<std::vector<double>(const Simplex&)> m_triangle_distance_edge_length;
    std::function<std::vector<long>(const Simplex&)> m_valence_improvement;


public:
    // constructor
    ATOperations(ATData& atdata, double target_distance, int64_t target_triangle_number);

    int64_t AT_rgb_split();
    int64_t AT_rgb_swap();
    ///// update
    void set_uvmesh_xyz_update_rule();
    void initialize_xyz();
    void set_distance_error_update_rule();
    void initialize_distance_error();
    void set_curved_edge_length_update_rule();
    void initialize_curved_edge_length();

    static double curved_edge_length_on_displaced_surface(
        const Eigen::Vector2d& uv0,
        const Eigen::Vector2d& uv1,
        const std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
            m_evaluator_ptr);
};
} // namespace wmtk::components::operations::internal