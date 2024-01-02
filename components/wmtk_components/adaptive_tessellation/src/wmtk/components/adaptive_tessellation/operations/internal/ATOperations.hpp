#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include "ATData.hpp"
namespace wmtk::components::adaptive_tessellation::operations::internal {
class ATOperations
{
public:
    ATData& m_atdata;
    std::vector<std::shared_ptr<wmtk::operations::Operation>> m_ops;
    double m_target_edge_length;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_edge_length_update;
    Accessor<double> m_edge_length_accessor;
    std::function<std::vector<double>(const Simplex&)> m_long_edges_first;
    std::function<std::vector<double>(const Simplex&)> m_short_edges_first;

public:
    // constructor
    ATOperations(ATData& atdata, double target_edge_length);

    void AT_smooth_interior();
    void AT_split_interior();
};
} // namespace wmtk::components::adaptive_tessellation::operations::internal