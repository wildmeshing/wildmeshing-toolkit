#include "surface_io.hpp"

#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>

namespace wmtk::components::surface_map {

namespace {

MeshData read(const std::string& path)
{
    try {
        return io::read_mesh(path);
    } catch (const std::exception& e) {
        log_and_throw_error("Could not read {}: {}", path, e.what());
    }
}

/// A field's rows, of `n` components each, as one matrix of the type it has in a mesh.
AttributeValues values_of(
    const std::vector<std::vector<double>>& field,
    const size_t n,
    const size_t type,
    const std::string& name)
{
    MeshAttribute a;
    DataMatrix<double> m(field.size(), n);
    for (size_t i = 0; i < field.size(); ++i) {
        if (field[i].size() != n) {
            log_and_throw_error(
                "field {}: {} components on one face, {} on another",
                name,
                n,
                field[i].size());
        }
        for (size_t j = 0; j < n; ++j) m(i, j) = field[i][j];
    }
    a.values = std::move(m);
    if (type == 1) a.values = a.integers();
    if (type == 2) {
        const DataMatrix<int64_t> i = a.integers();
        if ((i.array() < 0).any() || (i.array() > 255).any()) {
            log_and_throw_error("field {}: a value does not fit its UInt8 type", name);
        }
        a.values = DataMatrix<uint8_t>(i.cast<uint8_t>());
    }
    return a.values;
}

/// `s`'s fields as triangle attributes, rows in face order.
MeshAttributes attributes_of(const SurfaceData& s)
{
    MeshAttributes out;
    for (const auto* fields : {&s.face_fields, &s.corner_fields}) {
        for (const auto& [name, field] : *fields) {
            const size_t n = field.empty() ? 0 : field.front().size();
            if (field.size() != s.faces.size() || n == 0) {
                log_and_throw_error("field {} does not hold a value for every face", name);
            }
            const auto t = s.field_types.find(name);
            MeshAttribute& a = out[name];
            a.values = values_of(field, n, t == s.field_types.end() ? 0 : t->second.type, name);
            if (t != s.field_types.end()) a.labels = t->second.labels;
        }
    }
    return out;
}

} // namespace

bool is_corner_field(const std::string& name)
{
    return io::ComponentIO::reader_corner_attributes.count(name) > 0;
}

SurfaceData surface_from_mesh(const MeshData& mesh, const std::string& what, const bool strict)
{
    mesh.validate();
    for (const auto& [name, a] : mesh.vertex_attributes) {
        if (io::ComponentIO::reader_vertex_attributes.count(name)) continue;
        if (strict) {
            throw std::runtime_error(
                "surface_map: " + what + " has the vertex field " + name +
                ", which a transfer does not carry: it carries the fields of the triangles");
        }
        logger().warn(
            "{}: the vertex field {} is not carried, only fields of the triangles",
            what,
            name);
    }
    for (const auto& [type, c] : mesh.cells) {
        if (type == CellType::Triangle || c.connectivity.rows() == 0) continue;
        if (strict) {
            throw std::runtime_error(
                "surface_map: " + what +
                " holds cells other than triangles, which a transfer "
                "does not read");
        }
        logger().warn("{}: only its triangles are read", what);
    }
    const auto it = mesh.cells.find(CellType::Triangle);
    if (it == mesh.cells.end() || it->second.connectivity.rows() == 0) {
        log_and_throw_error("{} holds no triangles", what);
    }
    const CellData& triangles = it->second;
    SurfaceData s;
    s.vertices.resize(mesh.V.rows());
    for (Eigen::Index v = 0; v < mesh.V.rows(); ++v) s.vertices[v] = mesh.V.row(v).transpose();
    s.faces.resize(triangles.connectivity.rows());
    for (Eigen::Index f = 0; f < triangles.connectivity.rows(); ++f) {
        for (int k = 0; k < 3; ++k) s.faces[f][k] = uint64_t(triangles.connectivity(f, k));
    }
    for (const auto& [name, a] : triangles.attributes) {
        const DataMatrix<double> values = a.doubles();
        const bool corner = is_corner_field(name);
        if (corner && values.cols() % 3 != 0) {
            log_and_throw_error(
                "{}: {} holds a value per corner, but has {} columns, not three per corner",
                what,
                name,
                values.cols());
        }
        auto& field = (corner ? s.corner_fields : s.face_fields)[name];
        field.resize(values.rows());
        for (Eigen::Index f = 0; f < values.rows(); ++f) {
            field[f].assign(values.row(f).data(), values.row(f).data() + values.cols());
        }
        if (a.values.index() != 0 || !a.labels.empty()) {
            s.field_types[name] = {a.values.index(), a.labels};
        }
    }
    return s;
}

MeshData mesh_from_surface(const SurfaceData& s)
{
    MeshData mesh;
    mesh.V.resize(s.vertices.size(), 3);
    for (size_t v = 0; v < s.vertices.size(); ++v) mesh.V.row(v) = s.vertices[v].transpose();
    CellData& triangles = mesh.cells[CellType::Triangle];
    triangles.connectivity.resize(s.faces.size(), 3);
    for (size_t f = 0; f < s.faces.size(); ++f) {
        for (int k = 0; k < 3; ++k) triangles.connectivity(f, k) = int64_t(s.faces[f][k]);
    }
    triangles.attributes = attributes_of(s);
    mesh.validate();
    return mesh;
}

SurfaceData read_surface_msh(const std::string& path)
{
    return surface_from_mesh(read(path), path, false);
}

SurfaceData read_surface(const std::vector<std::string>& paths)
{
    if (paths.size() == 1 && std::filesystem::path(paths[0]).extension() == ".msh") {
        return read_surface_msh(paths[0]);
    }
    MatrixXd V;
    MatrixXi F;
    io::ReadProvenance read;
    io::read_triangle_mesh(paths, V, F, -1, -1, nullptr, &read);
    SurfaceData s;
    for (Eigen::Index v = 0; v < read.V.rows(); ++v)
        s.vertices.push_back(read.V.row(v).transpose());
    for (Eigen::Index f = 0; f < read.F.rows(); ++f) {
        s.faces.push_back(
            {{uint64_t(read.F(f, 0)), uint64_t(read.F(f, 1)), uint64_t(read.F(f, 2))}});
    }
    return s;
}

void write_surface_msh(const std::string& path, const SurfaceData& s)
{
    io::write_mesh(path, mesh_from_surface(s));
}

void add_surface(
    MeshData& mesh,
    const SurfaceData& s,
    const std::vector<int64_t>& rows,
    const std::string& what)
{
    // The surface must be where its vertices are in the mesh: the mesh the history ends at.
    if (rows.size() != s.vertices.size()) {
        log_and_throw_error(
            "{}: {} vertex rows for a surface of {}",
            what,
            rows.size(),
            s.vertices.size());
    }
    size_t missing = 0, misplaced = 0;
    for (size_t v = 0; v < rows.size(); ++v) {
        if (rows[v] < 0 || rows[v] >= mesh.V.rows()) {
            ++missing;
        } else if (mesh.V.row(rows[v]).transpose() != s.vertices[v]) {
            ++misplaced;
        }
    }
    if (missing + misplaced > 0) {
        log_and_throw_error(
            "{} is not the mesh the history ends at: of the surface's {} vertices, {} are not "
            "among its vertices and {} are elsewhere",
            what,
            s.vertices.size(),
            missing,
            misplaced);
    }

    // Each face onto the row of the triangle block with its corners, or a row after them.
    CellData& triangles = mesh.cells[CellType::Triangle];
    if (triangles.connectivity.cols() != 3) triangles.connectivity.resize(0, 3);
    std::map<std::array<int64_t, 3>, Eigen::Index> row_of;
    for (Eigen::Index r = 0; r < triangles.connectivity.rows(); ++r) {
        std::array<int64_t, 3> key;
        for (int k = 0; k < 3; ++k) key[k] = triangles.connectivity(r, k);
        std::sort(key.begin(), key.end());
        row_of.emplace(key, r);
    }
    const Eigen::Index n0 = triangles.connectivity.rows();
    std::vector<Eigen::Index> target(s.faces.size());
    std::vector<std::array<int64_t, 3>> appended;
    for (size_t f = 0; f < s.faces.size(); ++f) {
        std::array<int64_t, 3> t;
        for (int k = 0; k < 3; ++k) t[k] = rows[s.faces[f][k]];
        std::array<int64_t, 3> key = t;
        std::sort(key.begin(), key.end());
        const auto it = row_of.find(key);
        if (it != row_of.end()) {
            target[f] = it->second;
            continue;
        }
        target[f] = n0 + Eigen::Index(appended.size());
        row_of.emplace(key, target[f]);
        appended.push_back(t);
    }
    const Eigen::Index n = n0 + Eigen::Index(appended.size());
    triangles.connectivity.conservativeResize(n, 3);
    for (size_t i = 0; i < appended.size(); ++i) {
        for (int k = 0; k < 3; ++k) triangles.connectivity(n0 + i, k) = appended[i][k];
    }
    // The rows added have no value of the fields the block had: zero, an untagged face.
    for (auto& [name, a] : triangles.attributes) {
        a.values = std::visit(
            [&](const auto& m) -> AttributeValues {
                using M = std::decay_t<decltype(m)>;
                M out = M::Zero(n, m.cols());
                out.topRows(m.rows()) = m;
                return out;
            },
            a.values);
    }

    // The fields, each face's values on its row, a corner's at that corner of the row; a row no
    // face is on takes the value transfer_fields gives a face with no source.
    for (auto& [name, a] : attributes_of(s)) {
        if (triangles.attributes.count(name)) {
            log_and_throw_error("{}: its triangles already have a field {}", what, name);
        }
        const bool corner = s.corner_fields.count(name) > 0;
        const double none = name == "input_face" ? -1 : 0;
        a.values = std::visit(
            [&](const auto& m) -> AttributeValues {
                using M = std::decay_t<decltype(m)>;
                M out = M::Constant(n, m.cols(), typename M::Scalar(none));
                const Eigen::Index k = m.cols() / 3;
                for (size_t f = 0; f < s.faces.size(); ++f) {
                    const Eigen::Index r = target[f];
                    if (!corner) {
                        out.row(r) = m.row(f);
                        continue;
                    }
                    for (int c = 0; c < 3; ++c) {
                        for (int j = 0; j < 3; ++j) {
                            if (rows[s.faces[f][j]] != triangles.connectivity(r, c)) continue;
                            out.block(r, c * k, 1, k) = m.block(f, j * k, 1, k);
                        }
                    }
                }
                return out;
            },
            a.values);
        triangles.attributes[name] = std::move(a);
    }
    mesh.validate();
}

void write_surface_into_msh(
    const std::string& tet_mesh,
    const std::string& path,
    const SurfaceData& s,
    const std::vector<int64_t>& rows)
{
    MeshData mesh = read(tet_mesh);
    add_surface(mesh, s, rows, tet_mesh);
    io::write_mesh(path, mesh);
}

void write_surface_obj(
    const std::string& path,
    const SurfaceData& s,
    const std::string& uv_field,
    const std::string& texture)
{
    const auto it = s.corner_fields.find(uv_field);
    if (it == s.corner_fields.end())
        log_and_throw_error("{}: no per-corner field {}", path, uv_field);
    const auto& uv = it->second;
    const size_t k = uv.empty() ? 0 : uv.front().size() / 3;
    if (k < 2) log_and_throw_error("{}: {} has fewer than two components", path, uv_field);

    std::filesystem::path mtl = path;
    mtl.replace_extension(".mtl");
    {
        std::ofstream m(mtl);
        m << "newmtl surface\nKa 1 1 1\nKd 1 1 1\nmap_Kd " << texture << "\n";
    }
    std::ofstream o(path);
    if (!o) log_and_throw_error("Could not write {}", path);
    o.precision(17);
    o << "mtllib " << mtl.filename().string() << "\nusemtl surface\n";
    for (const auto& v : s.vertices) o << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
    for (const auto& f : uv) {
        for (int c = 0; c < 3; ++c) o << "vt " << f[c * k] << " " << f[c * k + 1] << "\n";
    }
    for (size_t i = 0; i < s.faces.size(); ++i) {
        o << "f";
        for (int c = 0; c < 3; ++c) o << " " << s.faces[i][c] + 1 << "/" << 3 * i + c + 1;
        o << "\n";
    }
}

} // namespace wmtk::components::surface_map
