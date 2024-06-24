#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Mesh;
}
namespace wmtk::operations {

/// Given a global id returns global ids
class SplitAlternateFacetData
{
public:
    using AltData = std::vector<std::tuple<int64_t, std::array<int64_t, 2>>>;
    AltData m_facet_maps;

    void sort();

    // assumes the split facet map has been sorted
    const std::array<int64_t, 2>& get_alternative_facets(const int64_t& input_facet) const;

    AltData::const_iterator get_alternative_facets_it(const int64_t& input_facet) const;
};

class CollapseAlternateFacetData
{
public:
    class Data
    {
        Data(const Mesh& m, const Tuple& input_tuple);
        Tuple input;
        std::array<Tuple, 2> alts;

    private:
        static Tuple left_switches(const Mesh& m, const Tuple& t);
        static Tuple right_switches(const Mesh& m, const Tuple& t);
    };

    void add(const Mesh& m, const Tuple& input_tuple) const;

    using AltData = std::vector<Data>;
    AltData m_data;

    void sort();

    // assumes the split facet map has been sorted
    const std::array<int64_t, 2>& get_alternative_facets(const int64_t& input_facet) const;

    AltData::const_iterator get_alternative_facets_it(const int64_t& input_facet) const;
};

class EdgeOperationData
{
public:
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

    SplitAlternateFacetDAta m_alternate_facetss;

protected:
    static Tuple tuple_from_id(const Mesh& m, const PrimitiveType type, const int64_t gid);
};
} // namespace wmtk::operations
