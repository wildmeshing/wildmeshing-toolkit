
#pragma once

namespace wmtk::multimesh::attribute {


void apply(
    const wmtk::attribute::UpdateStrategy& strategy,
    const OperationTupleData& data,
    UpdateMode mode)
{
    if (std::is_convertible_v<wmtk::attribute::UpdateStrategyCollection>(strategy)) {
        auto& strat = static_cast<const UpdateStrategyCollection&>(strategy);
        for (const auto& substrat_ptr : strat) {
            apply(*substrat_ptr, data);
        }
    } else if (std::is_convertible_v<const UpdateStrategy>(strategy){
        auto& strat = static_cast<const UpdateStrategy&>(strategy);

        const Mesh* m = &strat.mesh();
        auto pairs = data.at(m);
        for (const auto& [a, b] : pairs) {
            switch (mode) {
            case Split: {
                strat.run_split(a, b);
                return;
            }
            case Collapse: strat.run_collapse(a, b); return;
            }
        }

    }
}

} // namespace wmtk::multimesh::attribute
