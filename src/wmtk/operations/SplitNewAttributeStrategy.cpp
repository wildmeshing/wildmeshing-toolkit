#include "SplitNewAttributeStrategy.hpp"
#include <wmtk/utils/primitive_range.hpp>


namespace wmtk::operations {


SplitNewAttributeStrategy::update(const ReturnData& data, const OperationTupleData& op_datas) const
{

    assert(op_datas.find(&mesh()) != op_datas.end());
    const std::vector<std::array<Tuple, 2>>& tuple_pairs = op_datas.get(&mesh());

    for (const auto& tuple_pair : tuple_pairs) {
        const Tuple& input_tuple = tuple_pair[0];
        const Tuple& output_tuple = tuple_pair[1];

        const auto& return_data_variant =
            data.get_variant(mesh(), wmtk::simplex::Simplex::edge(input_tuple));

        for (const PrimitiveType pt : primitive_below(mesh().top_simplex_type())) {
        }
    }
}
} // namespace wmtk::operations
