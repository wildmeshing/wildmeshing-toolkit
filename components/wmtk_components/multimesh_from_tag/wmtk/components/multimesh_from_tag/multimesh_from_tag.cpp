#include "multimesh_from_tag.hpp"

#include <deque>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/utils/Logger.hpp>

#include "internal/MultiMeshFromTag.hpp"
#include "internal/MultiMeshFromTagOptions.hpp"

namespace wmtk {
namespace components {

using namespace internal;

void multimesh_from_tag(TriMesh& m, const MultiMeshFromTagOptions& options)
{
    attribute::MeshAttributeHandle substructure_label =
        base::get_attribute(m, options.substructure_label);
    const auto substructure_acc = m.create_const_accessor<int64_t>(substructure_label);

    const PrimitiveType substructure_ptype = substructure_label.primitive_type();
    const int64_t substructure_value = options.substructure_value;

    assert(substructure_ptype == PrimitiveType::Triangle); // start by implementing only the simple
                                                           // 2d triangle substructure case

    // create attributes to store new ids
    auto v_id_handle =
        m.register_attribute<int64_t>("new_v_ids", PrimitiveType::Vertex, 3, false, -1);
    auto v_id_acc = m.create_accessor<int64_t>(v_id_handle);

    const auto triangles = m.get_all(PrimitiveType::Triangle);

    // set vertex ids
    for (const Tuple& tri_tuple : triangles) {
        if (substructure_acc.const_scalar_attribute(tri_tuple) != substructure_value) {
            continue;
        }

        const auto vertex_tuples = simplex::faces_single_dimension_tuples(
            m,
            simplex::Simplex::face(tri_tuple),
            PrimitiveType::Vertex);

        for (const Tuple& v_tuple : vertex_tuples) {
            const int8_t& local_v_id = v_tuple.local_vid();
            if (v_id_acc.const_vector_attribute(v_tuple)[local_v_id] != -1) {
                continue;
            }
            //
        }
    }
}

void multimesh_from_tag(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    MultiMeshFromTagOptions options = j.get<MultiMeshFromTagOptions>();

    auto mesh_in = cache.read_mesh(options.input);

    Mesh& mesh = static_cast<Mesh&>(*mesh_in);

    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Triangle: multimesh_from_tag(static_cast<TriMesh&>(mesh), options); break;
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge:
    case PrimitiveType::Tetrahedron:
    default: log_and_throw_error("No implementation for the given mesh type");
    }


    cache.write_mesh(*mesh_in, options.output);
}
} // namespace components
} // namespace wmtk
