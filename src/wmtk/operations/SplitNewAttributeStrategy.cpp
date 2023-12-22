#include "SplitNewAttributeStrategy.hpp"
#include <wmtk/utils/primitive_range.hpp>


namespace wmtk::operations {


void SplitNewAttributeStrategy::update(const ReturnData& data, const OperationTupleData& op_datas)
{
    assert(op_datas.find(&mesh()) != op_datas.end());
    const std::vector<std::array<Tuple, 2>>& tuple_pairs = op_datas.at(&mesh());

    for (const auto& tuple_pair : tuple_pairs) {
        const Tuple& input_tuple = tuple_pair[0];
        const Tuple& output_tuple = tuple_pair[1];

        const auto& return_data_variant =
            data.get_variant(mesh(), wmtk::simplex::Simplex::edge(input_tuple));

        for (const PrimitiveType pt : wmtk::utils::primitive_below(mesh().top_simplex_type())) {
            {
                auto old_simps = input_ear_simplices(return_data_variant, input_tuple, pt);
                auto new_simps = output_rib_simplices(return_data_variant, output_tuple, pt);


                assert(old_simps.size() == new_simps.size());

                for (size_t s = 0; s < old_simps.size(); ++s) {
                    assign_split_ribs(pt, old_simps[s], new_simps[s]);
                }
                continue;
            }
            {
                auto old_simps = input_split_simplices(return_data_variant, input_tuple, pt);
                auto new_simps = output_split_simplices(return_data_variant, output_tuple, pt);


                assert(old_simps.size() == new_simps.size());

                for (size_t s = 0; s < old_simps.size(); ++s) {
                    assign_split(pt, old_simps[s], new_simps[s]);
                }
            }
        }
    }
}

} // namespace wmtk::operations
