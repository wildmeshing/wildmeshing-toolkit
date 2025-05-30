#pragma once
#include <array>
#include <memory>
#include <variant>
#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk {
class Mesh;
}
namespace wmtk::operations {

namespace internal {
class SplitAlternateFacetData;
class CollapseAlternateFacetData;
} // namespace internal

class EdgeOperationData
{
public:
    friend class internal::SplitAlternateFacetData;
    friend class internal::CollapseAlternateFacetData;
    EdgeOperationData();
    ~EdgeOperationData();
    EdgeOperationData(EdgeOperationData&&);
    EdgeOperationData& operator=(EdgeOperationData&&);
    Tuple m_operating_tuple;

    Tuple m_output_tuple;
    std::array<int64_t, 2> m_spine_vids; // two endpoints of the edge


    std::vector<std::vector<Tuple>> split_boundary_complex;

    // for multimesh we need to know which global ids are modified to trigger
    // for every simplex dimension (We have 3 in trimesh):
    // a list of [simplex index, {all versions of that simplex}]
    std::vector<std::vector<std::tuple<int64_t, std::vector<Tuple>>>>
        global_ids_to_potential_tuples;

    std::vector<std::vector<int64_t>> global_ids_to_update;

    // std::unique_ptr<internal::SplitAlternateFacetData> m_split_data;
    // std::unique_ptr<internal::CollapseAlternateFacetData> m_collapse_data;

    // std::variant<
    //     std::unique_ptr<internal::SplitAlternateFacetData>,
    //     std::unique_ptr<internal::CollapseAlternateFacetData>>
    //     m_op_data;


    void set_split();
    void set_collapse();


    /// Returns facet data held if the edge operation was a split - throws if data does not exist
    const internal::SplitAlternateFacetData& const_split_facet_data() const;
    /// Returns facet data held if the edge operation was a collapse- throws if data does not exist
    const internal::CollapseAlternateFacetData& const_collapse_facet_data() const;

protected:
    /// Returns facet data held if the edge operation was a split - throws if data does not exist
    internal::SplitAlternateFacetData& split_facet_data();
    /// Returns facet data held if the edge operation was a collapse- throws if data does not exist
    internal::CollapseAlternateFacetData& collapse_facet_data();

protected:
    static Tuple tuple_from_id(const Mesh& m, const PrimitiveType type, const int64_t gid);
    static simplex::Simplex
    simplex_from_id(const Mesh& m, const PrimitiveType type, const int64_t gid);
    static std::vector<int64_t>
    request_simplex_indices(Mesh& mesh, const PrimitiveType type, int64_t count);
};
} // namespace wmtk::operations
