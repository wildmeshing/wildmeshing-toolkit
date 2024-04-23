#include "input.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/base/resolve_path.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include "internal/InputOptions.hpp"
#include "wmtk/Mesh.hpp"
#include "wmtk/TetMesh.hpp"
#include "wmtk/attribute/MeshAttributeHandle.hpp"

namespace wmtk::components {

void input(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    InputOptions options = j.get<InputOptions>();

    std::string file = wmtk::components::base::resolve_path(options.file.string(), paths.root_path);

    if (!std::filesystem::exists(file)) {
        throw std::runtime_error(std::string("file") + file + " not found");
    }

    std::shared_ptr<Mesh> mesh = read_mesh(file, options.ignore_z, options.tetrahedron_attributes);
    assert(mesh->is_connectivity_valid());

    {
        // mesh->consolidate();
        // auto tet_handle = mesh->get_attribute_handle<int64_t>("tag", PrimitiveType::Tetrahedron);
        // auto face_handle = mesh->register_attribute<int64_t>("surface", PrimitiveType::Triangle,
        // 1); auto acc_tet = mesh->create_accessor<int64_t>(tet_handle); auto acc_face =
        // mesh->create_accessor<int64_t>(face_handle); for (const Tuple& f :
        // mesh->get_all(PrimitiveType::Triangle)) {
        //     if (!mesh->is_boundary(simplex::Simplex(PrimitiveType::Triangle, f))) {
        //         if (acc_tet.scalar_attribute(f) !=
        //             acc_tet.scalar_attribute(mesh->switch_tuple(f, PrimitiveType::Tetrahedron)))
        //             { acc_face.scalar_attribute(f) = 1;
        //         }
        //     }
        // }
        // multimesh::utils::extract_and_register_child_mesh_from_tag(
        //     *mesh,
        //     "surface",
        //     1,
        //     PrimitiveType::Triangle);

        // mesh->consolidate();
        // auto tag_handle = mesh->get_attribute_handle<int64_t>("tag", PrimitiveType::Tetrahedron);
        // auto face_handle = mesh->register_attribute<int64_t>("surface", PrimitiveType::Triangle,
        // 1); auto acc_face = mesh->create_accessor<int64_t>(face_handle); auto acc_tag =
        // mesh->create_accessor<int64_t>(tag_handle);

        // for (const Tuple& face : mesh->get_all(wmtk::PrimitiveType::Triangle)) {
        //     if (!mesh->is_boundary(simplex::Simplex(PrimitiveType::Triangle, face))) {
        //         if (acc_tag.scalar_attribute(face) !=
        //             acc_tag.scalar_attribute(
        //                 mesh->switch_tuple(face, PrimitiveType::Tetrahedron))) {
        //             acc_face.scalar_attribute(face) = 1;
        //         }
        //     }
        // }


        // {
        //     TetMesh& tetmesh = static_cast<TetMesh&>(*mesh);
        //     auto child_mesh = wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
        //         tetmesh,
        //         "surface",
        //         1,
        //         wmtk::PrimitiveType::Triangle);

        // std::shared_ptr<TetMesh> child_ptr =
        //     std::make_shared<TetMesh>(std::move(position_mesh));

        // names["periodic"] = mesh->absolute_multi_mesh_id();
        // names["surface"] = child_mesh->absolute_multi_mesh_id();

        //     cache.write_mesh(*mesh, "surface");
        // }

        // {
        //     auto child_mesh = multimesh::utils::extract_and_register_child_mesh_from_tag(
        //         mesh,
        //         "face_constraint_tag",
        //         1,
        //         wmtk::PrimitiveType::Triangle);
        //     HDF5Writer writer(output_file + ".constraint.hdf5");
        //     child_mesh->serialize(writer);
        // }
    }


    cache.write_mesh(*mesh, options.name);
}
} // namespace wmtk::components