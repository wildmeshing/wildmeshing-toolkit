#pragma once

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <wmtk/MeshData.hpp>

#include <cstring>
#include <string>

namespace wmtk::python {
using MemoryComponent = ComponentResult (*)(nlohmann::json, const MeshMap&);

// NumPy arrays and mesh dictionaries to and from MeshData, for the module and the components'
// own bindings (app/pywildmeshing/<name>_bindings.cpp). Every array is copied: NumPy owns what
// is returned, and nothing keeps a reference into what is given.

/// `value` as a dictionary; a TypeError naming it otherwise.
pybind11::dict dictionary(pybind11::handle value, const std::string& name);
/// `value` as a NumPy array; a TypeError otherwise.
pybind11::array array(pybind11::handle value);
/// A copy of a two-dimensional array -- or, with `allow_vector`, a one-dimensional one, as one
/// column -- converted to T.
template <typename T>
DataMatrix<T> matrix(const pybind11::array& value, bool allow_vector = false)
{
    if (value.ndim() != 2 && !(allow_vector && value.ndim() == 1))
        throw pybind11::value_error(
            "Mesh arrays must have two dimensions (attributes may be vectors)");
    const auto contiguous =
        pybind11::array_t<T, pybind11::array::c_style | pybind11::array::forcecast>::ensure(value);
    if (!contiguous) throw pybind11::type_error("Cannot convert mesh array");
    DataMatrix<T> result(value.shape(0), value.ndim() == 1 ? 1 : value.shape(1));
    if (result.size())
        std::memcpy(result.data(), contiguous.data(), size_t(result.size()) * sizeof(T));
    return result;
}
/// An array of an integer dtype, as Int64: a TypeError for any other dtype, a ValueError for a
/// value beyond Int64. One-dimensional too with `allow_vector`, as for matrix().
DataMatrix<int64_t> integers(const pybind11::array& value, bool allow_vector = false);
/// An owning NumPy array of the matrix's shape.
template <typename T>
pybind11::array numpy(const DataMatrix<T>& matrix)
{
    pybind11::array_t<T> result({matrix.rows(), matrix.cols()});
    if (matrix.size())
        std::memcpy(result.mutable_data(), matrix.data(), size_t(matrix.size()) * sizeof(T));
    return result;
}
/// A mesh dictionary -- V, cells, vertex_attributes, as docs/in-memory-io.md has them -- and
/// back.
MeshData mesh_data(const pybind11::dict& input);
pybind11::dict mesh_data(const MeshData& input);

// pybind11_json turns bytes into base64 text and has no binary values: options() makes the bytes
// in a call's options (a history) JSON binary values, and report() those of a report
// (the history a memory call returns) bytes again. Everything else converts as pybind11_json does.
nlohmann::json options(const pybind11::handle& value);
pybind11::object report(const nlohmann::json& j);
pybind11::tuple
call_component(MemoryComponent component, nlohmann::json options, const pybind11::dict& meshes);
void bind_mesh_io(pybind11::module_& module);
} // namespace wmtk::python
