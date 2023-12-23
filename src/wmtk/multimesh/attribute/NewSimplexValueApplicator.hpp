
#pragma once
#include <wmtk/multimesh/operations/CollapseReturnData.hpp>
#include <wmtk/multimesh/operations/SplitReturnData.hpp>
#include <wmtk/attribute/SmartAttributeHandle.hpp>

namespace wmtk::multimesh::attribute {


class NewSimplexValueApplicator: public SplitNewValueApplicator, CollapseNewValueApplicator
{

    public:
        void apply(const wmtk::multimesh::operations::SplitReturnData&, const Tuple& input_tuple, const Tuple& output_tuple);
        void apply(const wmtk::multimesh::operations::CollapseReturnData&, const Tuple& input_tuple, const Tuple& output_tuple);
    private:

};


} // namespace wmtk::multimesh::attribute
