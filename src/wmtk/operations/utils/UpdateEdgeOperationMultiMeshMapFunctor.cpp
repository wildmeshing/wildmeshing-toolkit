#include "UpdateEdgeOperationMultiMeshMapFunctor.hpp"
//#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>


namespace wmtk::operations::utils {
void UpdateEdgeOperationMultiMeshMapFunctor::operator()(
    TriMesh& parent_mesh,
    const TriMeshOperationExecutor& parent_tmoe,
    TriMesh& child_mesh,
    const TriMeshOperationExecutor& child_tmoe)
{
    const auto& parent_incident_data = parent_tmoe.incident_face_datas();
    const auto& child_incident_data = child_tmoe.incident_face_datas();

    assert(parent_incident_data.size() == cihld_indcident_data.size());

    auto& parent_mmmanager = parent_mesh.m_multi_mesh_manager;
    auto& child_mmmanager = child_mesh.m_multi_mesh_manager;

    auto child_to_parent_handle = child_mmmanager.map_to_parent_handle;

    long child_id = child_mesh.multi_mesh_manager.child_id();
    auto parent_to_child_handle = parent_mmmanager.children().at(child_id).map_handle;
    auto child_to_parent_accessor = child_mesh.create_accessor(child_to_parent_handle);
    auto parent_to_child_accessor = my_mesh.create_accessor(parent_to_child_handle);

    using IncidentData = TriMeshOperationExecutor::IncidentFaceData;
    auto update_from_incident_data = [&](const IncidentData& parent_data,
                                         const IncidentData& child_data) {
        const auto child_split_f = child_data.split_f;
        const auto parent_split_f = parent_data.split_f;


        for (long index = 0; index < 2; ++index) {
            long f_child = child_split_f[index];
            long f_parent = parent_split_f[index];
            Tuple child_tuple = (f_child == -1) ? Tuple() : child_mesh.face_tuple_from_id(f_child);
            Tuple parent_tuple = m_mesh.face_tuple_from_id(f_parent);

            if (!tuple_child.is_null()) {
                multimesh::utils::write_tuple_map_attribute(
                    child_to_parent_accessor,
                    child_tuple,
                    parent_tuple);
            }
            multimesh::utils::write_tuple_map_attribute(
                parent_to_child_accessor,
                parent_tuple,
                child_tuple);
        }
    };

    // update_hash on new cells
    for (long i = 0; i < long(parent_incident_data.size()); i++) {
        update_from_incident_data(parent_incident_data[i], child_incident_data[i]);
    }

    // update_hash on neighboring cells
    update_hash_in_map(child_mesh);
}
} // namespace wmtk::operations::utils
