#include "HDF5Reader.hpp"

#include <wmtk/Mesh.hpp>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

#include <h5pp/h5pp.h>

#include <regex>


namespace wmtk {
HDF5Reader::HDF5Reader() {}


std::shared_ptr<Mesh> HDF5Reader::read(const std::filesystem::path& filename)
{
    constexpr static int64_t TWO_TUPLE_SIZE = 10;
    constexpr static int64_t DEFAULT_TUPLES_VALUES = -1;

    h5pp::File hdf5_file(filename, h5pp::FileAccess::READONLY);

    auto root = read_mesh(hdf5_file, "WMTK");

    if (hdf5_file.linkExists("WMTK/multimesh")) {
        std::map<std::vector<int64_t>, std::shared_ptr<Mesh>> meshes;
        meshes[{}] = root;

        const auto dsets = hdf5_file.findGroups("mesh_", "WMTK/multimesh", -1, 1);
        for (auto& s : dsets) {
            const std::string dataset = "WMTK/multimesh/" + s;

            auto absolute_id =
                hdf5_file.readAttribute<std::vector<int64_t>>(dataset, "absolute_id");
            meshes[absolute_id] = read_mesh(hdf5_file, dataset);
        }


        for (auto& p : meshes) {
            if (p.first.empty()) continue;

            const std::vector<int64_t>& child_id = p.first;
            const auto child_index = child_id.back();

            std::vector<int64_t> parent_id = child_id;
            parent_id.pop_back();

            auto parent_mesh = meshes.at(parent_id);
            auto child_mesh = p.second;

            const PrimitiveType child_primitive_type = child_mesh->top_simplex_type();


            assert(parent_mesh->m_multi_mesh_manager.m_children.size() == child_index);

            auto child_to_parent_handle = child_mesh->get_attribute_handle<int64_t>(
                MultiMeshManager::child_to_parent_map_attribute_name(),
                child_primitive_type).as<int64_t>();

            auto parent_to_child_handle = parent_mesh->get_attribute_handle<int64_t>(
                MultiMeshManager::parent_to_child_map_attribute_name(child_index),
                child_primitive_type).as<int64_t>();

            child_mesh->m_multi_mesh_manager.m_parent = parent_mesh.get();
            child_mesh->m_multi_mesh_manager.m_child_id = child_index;
            child_mesh->m_multi_mesh_manager.map_to_parent_handle = child_to_parent_handle;

            parent_mesh->m_multi_mesh_manager.m_children.emplace_back();
            parent_mesh->m_multi_mesh_manager.m_children.back().mesh = child_mesh;
            parent_mesh->m_multi_mesh_manager.m_children.back().map_handle = parent_to_child_handle;
        }

#if !defined(NDEBUG)
        for (auto& p : meshes) {
            assert(p.first == p.second->m_multi_mesh_manager.absolute_id());
        }
#endif
    }


    return root;
}

std::shared_ptr<Mesh> HDF5Reader::read_mesh(h5pp::File& hdf5_file, const std::string& root_dataset)
{
    PrimitiveType top_simplex_type =
        hdf5_file.readAttribute<PrimitiveType>(root_dataset, "top_simplex_type");

    std::shared_ptr<Mesh> mesh;

    switch (top_simplex_type) {
    case PrimitiveType::Vertex: mesh = std::make_shared<PointMesh>(); break;
    case PrimitiveType::HalfEdge:
    case PrimitiveType::Edge: mesh = std::make_shared<EdgeMesh>(); break;
    case PrimitiveType::Face: mesh = std::make_shared<TriMesh>(); break;
    case PrimitiveType::Tetrahedron: mesh = std::make_shared<TetMesh>(); break;
    default: break;
    }

    std::vector<int64_t> capacities =
        hdf5_file.readAttribute<std::vector<int64_t>>(root_dataset, "capacities");


    mesh->set_capacities(capacities);


    const auto dsets = hdf5_file.findDatasets("", root_dataset, -1, 1);
    for (auto& s : dsets) {
        const std::string dataset = root_dataset + "/" + s;


        const int64_t stride = hdf5_file.readAttribute<int64_t>(dataset, "stride");
        const int64_t dimension = hdf5_file.readAttribute<int64_t>(dataset, "dimension");
        const std::string type = hdf5_file.readAttribute<std::string>(dataset, "type");
        const std::string name =
            std::regex_replace(s, std::regex(std::to_string(dimension) + "/"), "");

        auto pt = PrimitiveType(dimension);

        if (type == "int64_t") {
            auto v = hdf5_file.readDataset<std::vector<int64_t>>(dataset);
            const auto default_val = hdf5_file.readAttribute<int64_t>(dataset, "default_value");

            set_attribute<int64_t>(default_val, name, pt, stride, v, *mesh);
        } else if (type == "char") {
            auto tmp = hdf5_file.readDataset<std::vector<short>>(dataset);
            const auto default_val = char(hdf5_file.readAttribute<short>(dataset, "default_value"));

            std::vector<char> v;
            v.reserve(tmp.size());
            for (auto val : tmp) v.push_back(char(val));


            set_attribute<char>(default_val, name, pt, stride, v, *mesh);
        } else if (type == "double") {
            auto v = hdf5_file.readDataset<std::vector<double>>(dataset);
            const auto default_val = hdf5_file.readAttribute<double>(dataset, "default_value");


            set_attribute<double>(default_val, name, pt, stride, v, *mesh);
        } else if (type == "rational") {
            auto tmp = hdf5_file.readDataset<std::vector<std::string>>(dataset);
            assert(tmp.size() % 2 == 0);
            std::string numer = hdf5_file.readAttribute<std::string>(dataset, "default_value");
            const auto default_val = Rational(numer, "1");

            std::vector<Rational> v;
            v.reserve(tmp.size() / 2);
            for (size_t i = 0; i < tmp.size(); i += 2) {
                v.emplace_back(tmp[i], tmp[i + 1]);
            }

            set_attribute<Rational>(default_val, name, pt, stride, v, *mesh);

        } else {
            logger().error("We currently do not support reading the type \"{}\"", type);
            assert(false);
        }
    }

    return mesh;
}

template <typename T>
void HDF5Reader::set_attribute(
    const T& default_val,
    const std::string& name,
    PrimitiveType pt,
    int64_t stride,
    const std::vector<T>& v,
    Mesh& mesh)
{
    attribute::MeshAttributeHandle handle =
        mesh.has_attribute<T>(name, pt)
            ? mesh.get_attribute_handle<T>(name, pt)
            : mesh.register_attribute<T>(name, pt, stride, false, default_val);
    const attribute::TypedAttributeHandle<T>& thandle = handle.as<T>();

    int64_t handle_dimension = mesh.get_attribute_dimension(thandle);
    if (stride != handle_dimension) {
        log_and_throw_error(
            "Attribute does not have the expected dimension:\n  expected {}\n  actual {}",
            stride,
            handle_dimension);
    }

    auto accessor = attribute::AccessorBase<T>(mesh, thandle);

    accessor.set_attribute(v);
}


} // namespace wmtk
