#pragma once
#include "IsotropicRemeshingOptions.hpp"


namespace wmtk {
namespace operations {
class EdgeCollapse;
class EdgeSplit;
class EdgeSwap;
class Operation;
class AttributesUpdateWithFunction;
} // namespace operations
namespace invariants {
class EnvelopeInvariant;
}
} // namespace wmtk
namespace wmtk::components::isotropic_remeshing {

class IsotropicRemeshing
{
public:
    IsotropicRemeshing(const IsotropicRemeshingOptions& opts);
    ~IsotropicRemeshing();


    void run();

private:
    std::vector<wmtk::attribute::MeshAttributeHandle> all_envelope_positions() const;
    static bool is_envelope_position(const wmtk::attribute::MeshAttributeHandle& position);

private:
    IsotropicRemeshingOptions m_options;


    std::shared_ptr<operations::EdgeSplit> m_split;
    std::shared_ptr<operations::EdgeCollapse> m_collapse;
    std::shared_ptr<operations::Operation> m_swap;
    std::shared_ptr<operations::AttributesUpdateWithFunction> m_smooth;

    std::vector<std::pair<std::string, std::shared_ptr<operations::Operation>>> m_operations;
    std::vector<std::shared_ptr<wmtk::invariants::EnvelopeInvariant>> m_envelope_invariants;

    std::shared_ptr<operations::EdgeSplit> configure_split();
    std::shared_ptr<operations::EdgeCollapse> configure_collapse();
    std::shared_ptr<operations::Operation> configure_swap();
    std::shared_ptr<operations::AttributesUpdateWithFunction> configure_smooth();
};

} // namespace wmtk::components::isotropic_remeshing
