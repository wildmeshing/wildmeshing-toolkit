
#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Mesh;
}
namespace wmtk::operations::internal {


class CollapseAlternateFacetData
{
public:
    class Data
    {
    public:
        Data(const Mesh& m, const Tuple& input_tuple);
        Tuple input;
        std::array<Tuple, 2> alts;
        std::array<char8_t, 2> boundary_local_index;

        std::array<std::vector<wmtk::Tuple>> sequences;

    private:
        static Tuple left_switches(const Mesh& m, const Tuple& t);
        static Tuple right_switches(const Mesh& m, const Tuple& t);
    };

    void add(const Mesh& m, const Tuple& input_tuple);

    std::array<Tuple, 2> get_alternatives(const PrimitiveType mesh_pt, const Tuple& t) const;
    Tuple get_alternative(const PrimitiveType mesh_pt, const Tuple& t) const;

    using AltData = std::vector<Data>;
    AltData m_data;

    void sort();

    const Data& get_alternatives_data(const Tuple& t) const;

    AltData::const_iterator get_alternative_data_it(const int64_t& input_facet) const;
};
} // namespace wmtk::operations::internal
