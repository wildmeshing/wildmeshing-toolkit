// gcc-13 complains about how we use an empty vector as a key.
// Vector should do empty key deref for us - we hope
#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
#endif
#include <map>
#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#include <string>
#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include "HDF5Reader.hpp"

#include <wmtk/Mesh.hpp>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

#include <wmtk/utils/tuple_map_attribute_io.hpp>

#include <h5pp/h5pp.h>

#include <regex>


namespace wmtk {
namespace {
template <typename T>
std::string get_type()
{
    return "";
}

template <>
std::string get_type<int64_t>()
{
    return "int64_t";
}

template <>
std::string get_type<double>()
{
    return "double";
}

template <>
std::string get_type<char>()
{
    return "char";
}
// template <>
// std::string get_type<short>()
//{
//     return "char";
// }

template <>
std::string get_type<wmtk::Rational>()
{
    return "rational";
}
// template <>
// std::string get_type<std::string>()
//{
//     return "rational";
// }
} // namespace

HDF5Reader::HDF5Reader() {}


std::shared_ptr<Mesh> HDF5Reader::read(const std::filesystem::path& filename)
{
    constexpr static int64_t TWO_TUPLE_SIZE = wmtk::utils::TWO_TUPLE_SIZE;
    constexpr static int64_t DEFAULT_TUPLES_VALUES = wmtk::utils::DEFAULT_TUPLES_VALUES;

    h5pp::File hdf5_file(filename, h5pp::FileAccess::READONLY);

    return read_mesh(hdf5_file, "WMTK");
}

std::shared_ptr<Mesh> HDF5Reader::read_mesh(h5pp::File& hdf5_file, const std::string& root_dataset)
{
    PrimitiveType top_simplex_type =
        hdf5_file.readAttribute<PrimitiveType>(root_dataset, "top_simplex_type");

    std::shared_ptr<Mesh> mesh;

    switch (top_simplex_type) {
    case PrimitiveType::Vertex: mesh = std::make_shared<PointMesh>(); break;
    case PrimitiveType::Edge: mesh = std::make_shared<EdgeMesh>(); break;
    case PrimitiveType::Triangle: mesh = std::make_shared<TriMesh>(); break;
    case PrimitiveType::Tetrahedron: mesh = std::make_shared<TetMesh>(); break;
    default: break;
    }

    std::vector<int64_t> capacities =
        hdf5_file.readAttribute<std::vector<int64_t>>(root_dataset, "capacities");


    mesh->set_capacities(capacities);

    {
        bool found_something = false;
        auto searcher = [&](int dim, auto&& t) {
            using T = std::decay_t<decltype(t)>;
            const static std::string name = get_type<T>();

            std::string attr_dset_name =
                fmt::format("{}/ATTRIBUTE_LIST_{}/{}", root_dataset, name, dim);
            try {
                auto names = hdf5_file.readDataset<std::vector<std::string>>(attr_dset_name);
                for (const auto& v : names) {
                    found_something = true;
                    const std::string dataset = fmt::format("{}/{}/{}", root_dataset, dim, v);
                    read_attribute(hdf5_file, *mesh, dataset, v);
                }
            } catch (const std::exception& e) {
            }
        };
        auto search = [&](int dim) {
            searcher(dim, double{});
            searcher(dim, wmtk::Rational{});
            searcher(dim, int64_t{});
            searcher(dim, char{});
        };
        switch (top_simplex_type) {
        case PrimitiveType::Tetrahedron: search(3); [[fallthrough]];
        case PrimitiveType::Triangle: search(2); [[fallthrough]];
        case PrimitiveType::Edge: search(1); [[fallthrough]];
        case PrimitiveType::Vertex: search(0); [[fallthrough]];
        default: break;
        }

        if (found_something) { // if we didnt find attribute lists assume we are using an old format
                               // mesh
            return mesh;
        }
    }


    const auto dsets = hdf5_file.findDatasets("", root_dataset, -1, 1);
    for (auto& s : dsets) {
        const std::string dataset = fmt::format("{}/{}", root_dataset, s);
        read_attribute(hdf5_file, *mesh, dataset, s);
    }

    return mesh;
}
void HDF5Reader::read_attribute(
    h5pp::File& hdf5_file,
    Mesh& mesh,
    const std::string& dataset,
    const std::string& name_)
{
    const int64_t stride = hdf5_file.readAttribute<int64_t>(dataset, "stride");
    const int64_t dimension = hdf5_file.readAttribute<int64_t>(dataset, "dimension");
    const std::string type = hdf5_file.readAttribute<std::string>(dataset, "type");
    const std::string name =
        std::regex_replace(name_, std::regex(std::to_string(dimension) + "/"), "");

    auto pt = PrimitiveType(dimension);

    if (type == "int64_t") {
        auto v = hdf5_file.readDataset<std::vector<int64_t>>(dataset);
        const auto default_val = hdf5_file.readAttribute<int64_t>(dataset, "default_value");

        set_attribute<int64_t>(default_val, name, pt, stride, v, mesh);
    } else if (type == "char") {
        auto tmp = hdf5_file.readDataset<std::vector<short>>(dataset);
        const auto default_val = char(hdf5_file.readAttribute<short>(dataset, "default_value"));

        std::vector<char> v;
        v.reserve(tmp.size());
        for (auto val : tmp) v.push_back(char(val));


        set_attribute<char>(default_val, name, pt, stride, v, mesh);
    } else if (type == "double") {
        auto v = hdf5_file.readDataset<std::vector<double>>(dataset);
        const auto default_val = hdf5_file.readAttribute<double>(dataset, "default_value");


        set_attribute<double>(default_val, name, pt, stride, v, mesh);
    } else if (type == "rational") {
        const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tmp =
            hdf5_file
                .readDataset<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                    dataset);
        const Eigen::Matrix<char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tmp_char =
            tmp.cast<char>();

        const std::string data = hdf5_file.readAttribute<std::string>(dataset, "default_value");
        Rational default_val;
        default_val.init_from_binary(data);

        std::vector<Rational> v;
        v.reserve(tmp.rows());
        for (size_t i = 0; i < tmp.rows(); ++i) {
            v.emplace_back(tmp_char.row(i));
        }

        set_attribute<Rational>(default_val, name, pt, stride, v, mesh);

    } else {
        logger().error("We currently do not support reading the type \"{}\"", type);
        assert(false);
    }
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

    auto accessor = mesh.create_accessor<T>(thandle);

    accessor.attribute().set(v);
}


} // namespace wmtk
