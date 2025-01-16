#include "utils.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/mesh_utils.hpp>



void EigenMeshes::compute_vf() {
    VF.resize(V.M.rows());
    for(int j = 0; j < F.M.rows(); ++j) {
        for(const auto& vid: F.M.row(j)) {
            VF[vid].emplace(j);
        }

    }
}

std::map<std::string, EigenMeshes> get_meshes(
    const wmtk::components::multimesh::MeshCollection& mc,
    const std::string_view& position_attribute_name)
{
    auto all_meshes = mc.all_roots();

    std::map<std::string, EigenMeshes> ranges;



    int total_V = 0;
    int total_F = 0;
    {
        for (const auto& [name, mesh] : all_meshes) {
            assert(mesh.is_connectivity_valid());
            wmtk::utils::EigenMatrixWriter writer;
            writer.set_position_attribute_name(position_attribute_name);
            mesh.serialize(writer);

            EigenMeshes em;
            em.V.m_start = total_V;
            em.F.m_start = total_F;



            writer.get_position_matrix(em.V.M);
            writer.get_FV_matrix(em.F.M);
            assert(em.F.M.maxCoeff() < em.V.M.rows());

            total_V = em.V.end();
            total_F = em.F.end();
            em.compute_vf();
            ranges[name] = std::move(em);
        }
    }

    return ranges;
}
