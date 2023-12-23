

#pragma once
#include <wmtk/multimesh/operations/CollapseReturnData.hpp>
#include <wmtk/multimesh/operations/SplitReturnData.hpp>

namespace wmtk::multimesh::attribute {


class UpdateStrategyCollection
{
    void update_new_simplex() const override;
};


} // namespace wmtk::multimesh::attribute
