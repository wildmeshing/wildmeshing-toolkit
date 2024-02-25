#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components::internal {

class MeshDecimation
{
public:
    MeshDecimation(
        Mesh& mesh,
        std::string constriant_name,
        int64_t constrait_value,
        double target_len,
        const std::vector<attribute::MeshAttributeHandle>& pass_through_attributes);

    void process();

private:
    Mesh& m_mesh;

    int64_t m_constrait_value;
    std::string m_constriant_name;
    double m_target_len;

    std::vector<attribute::MeshAttributeHandle> m_pass_through_attributes;
};

} // namespace wmtk::components::internal
