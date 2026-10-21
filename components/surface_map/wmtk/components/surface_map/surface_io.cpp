#include "surface_io.hpp"

#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>

#include <filesystem>
#include <fstream>
#include <limits>
#include <map>

namespace wmtk::components::surface_map {

namespace {
/// A single-block surface .msh (MshData::is_single_surface_block) with its fields; `msh` loaded.
SurfaceData surface_with_fields(MshData& msh, const std::string& path)
{
    SurfaceData s;
    s.vertices.resize(msh.get_num_face_vertices());
    s.faces.resize(msh.get_num_faces());
    msh.extract_face_vertices(
        [&s](size_t i, double x, double y, double z) { s.vertices[i] = Eigen::Vector3d(x, y, z); });
    msh.extract_faces([&s](size_t i, size_t a, size_t b, size_t c) { s.faces[i] = {{a, b, c}}; });
    if (s.faces.empty()) log_and_throw_error("{} holds no triangles", path);
    for (const std::string& name : msh.get_face_attribute_names()) {
        auto& field = s.face_fields[name];
        field.resize(s.faces.size());
        msh.extract_face_attribute(name, [&field](size_t i, const std::vector<double>& data) {
            if (i < field.size()) field[i] = data;
        });
    }
    const mshio::ElementBlock* triangles = &msh.m_spec.elements.entity_blocks.front();
    for (const mshio::Data& data : msh.m_spec.element_node_data) {
        if (data.header.string_tags.empty()) continue;
        auto& field = s.corner_fields[data.header.string_tags.front()];
        field.assign(s.faces.size(), {});
        const size_t offset = triangles->data.front();
        for (const mshio::DataEntry& entry : data.entries) {
            const size_t f = entry.tag - offset;
            if (f < field.size() && entry.num_nodes_per_element == 3) field[f] = entry.data;
        }
    }
    return s;
}
} // namespace

SurfaceData read_surface_msh(const std::string& path)
{
    MshData msh;
    try {
        msh.load(path);
    } catch (const std::exception& e) {
        log_and_throw_error("Could not read {}: {}", path, e.what());
    }
    if (!msh.is_single_surface_block()) {
        log_and_throw_error(
            "{} is not a single-block surface file, as wmtk writes one: its fields cannot be "
            "matched to its triangles",
            path);
    }
    return surface_with_fields(msh, path);
}

SurfaceData read_surface(const std::vector<std::string>& paths)
{
    if (paths.size() == 1 && std::filesystem::path(paths[0]).extension() == ".msh") {
        MshData msh;
        try {
            msh.load(paths[0]);
        } catch (const std::exception&) {
        }
        if (msh.is_single_surface_block()) return surface_with_fields(msh, paths[0]);
        // Read as the applications read it, so its faces are theirs; its fields are not.
        logger().warn(
            "{} is not a single-block surface file, as wmtk writes one: its fields are not read",
            paths[0]);
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

namespace {
/// The per-face and per-corner fields of `s` as gmsh data on the elements tagged `tags`.
void add_fields(
    MshData& msh,
    const SurfaceData& s,
    const std::vector<size_t>& tags,
    const std::string& path)
{
    for (const auto& [name, field] : s.face_fields) {
        const size_t n = field.empty() ? 0 : field.front().size();
        if (n != 1 && n != 3 && n != 9) {
            logger().warn("{}: field {} has {} components per face; not written", path, name, n);
            continue;
        }
        mshio::Data data;
        data.header.string_tags = {name};
        data.header.real_tags = {0.0};
        data.header.int_tags = {0, int(n), int(s.faces.size()), 0};
        data.entries.resize(s.faces.size());
        for (size_t i = 0; i < s.faces.size(); ++i) {
            data.entries[i].tag = tags[i];
            data.entries[i].data = field[i];
        }
        msh.m_spec.element_data.push_back(std::move(data));
    }
    for (const auto& [name, field] : s.corner_fields) {
        if (field.empty() || field.front().empty() || field.front().size() % 3 != 0) {
            logger().warn(
                "{}: per-corner field {} is empty or not 3 corners; not written",
                path,
                name);
            continue;
        }
        mshio::Data data;
        data.header.string_tags = {name};
        data.header.real_tags = {0.0};
        data.header.int_tags = {0, int(field.front().size() / 3), int(s.faces.size()), 0};
        data.entries.resize(s.faces.size());
        for (size_t i = 0; i < s.faces.size(); ++i) {
            data.entries[i].tag = tags[i];
            data.entries[i].num_nodes_per_element = 3;
            data.entries[i].data = field[i];
        }
        msh.m_spec.element_node_data.push_back(std::move(data));
    }
}
} // namespace

void write_surface_into_msh(
    const std::string& tet_mesh,
    const std::string& path,
    const SurfaceData& s)
{
    MshData msh;
    try {
        msh.load(tet_mesh);
    } catch (const std::exception& e) {
        log_and_throw_error("Could not read {}: {}", tet_mesh, e.what());
    }
    auto& spec = msh.m_spec;
    std::map<std::array<double, 3>, size_t> node_at;
    for (const auto& block : spec.nodes.entity_blocks) {
        const size_t stride = 3 + size_t(block.parametric) * size_t(block.entity_dim);
        for (size_t i = 0; i < block.num_nodes_in_block; ++i) {
            const double* p = &block.data[stride * i];
            node_at.emplace(std::array<double, 3>{{p[0], p[1], p[2]}}, block.tags[i]);
        }
    }
    std::vector<size_t> node(s.vertices.size());
    size_t missing = 0;
    for (size_t v = 0; v < s.vertices.size(); ++v) {
        const auto it = node_at.find({{s.vertices[v][0], s.vertices[v][1], s.vertices[v][2]}});
        if (it == node_at.end()) {
            ++missing;
        } else {
            node[v] = it->second;
        }
    }
    if (missing > 0) {
        log_and_throw_error(
            "{}: {} of the surface's {} vertices are not nodes of the tet mesh",
            tet_mesh,
            missing,
            s.vertices.size());
    }

    // The triangles, on a surface entity of their own.
    int entity = 0;
    for (const auto& b : spec.elements.entity_blocks) {
        if (b.entity_dim == 2) entity = std::max(entity, b.entity_tag);
    }
    for (const auto& e : spec.entities.surfaces) entity = std::max(entity, e.tag);
    ++entity;
    if (!spec.entities.empty()) {
        mshio::SurfaceEntity e;
        e.tag = entity;
        Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
        Eigen::Vector3d hi = -lo;
        for (const auto& v : s.vertices) {
            lo = lo.cwiseMin(v);
            hi = hi.cwiseMax(v);
        }
        e.min_x = lo[0];
        e.min_y = lo[1];
        e.min_z = lo[2];
        e.max_x = hi[0];
        e.max_y = hi[1];
        e.max_z = hi[2];
        spec.entities.surfaces.push_back(e);
    }
    mshio::ElementBlock block;
    block.entity_dim = 2;
    block.entity_tag = entity;
    block.element_type = 2; // 3-node triangle
    block.num_elements_in_block = s.faces.size();
    std::vector<size_t> tags(s.faces.size());
    for (size_t i = 0; i < s.faces.size(); ++i) {
        tags[i] = spec.elements.max_element_tag + i + 1;
        block.data.push_back(tags[i]);
        for (const uint64_t v : s.faces[i]) block.data.push_back(node[v]);
    }
    spec.elements.num_entity_blocks += 1;
    spec.elements.num_elements += s.faces.size();
    spec.elements.max_element_tag += s.faces.size();
    if (spec.elements.min_element_tag == 0) spec.elements.min_element_tag = 1;
    spec.elements.entity_blocks.push_back(std::move(block));
    add_fields(msh, s, tags, path);
    msh.save(path);
}

void write_surface_msh(const std::string& path, const SurfaceData& s)
{
    MshData msh;
    msh.add_face_vertices(s.vertices.size(), [&s](size_t i) { return s.vertices[i]; });
    msh.add_faces(s.faces.size(), [&s](size_t i) { return s.faces[i]; });
    for (const auto& [name, field] : s.face_fields) {
        const size_t n = field.empty() ? 0 : field.front().size();
        if (n == 1) {
            msh.add_face_attribute<1>(name, [&field](size_t i) { return field[i][0]; });
        } else if (n == 3) {
            msh.add_face_attribute<3>(name, [&field](size_t i) {
                return Eigen::Vector3d(field[i][0], field[i][1], field[i][2]);
            });
        } else if (n == 9) {
            msh.add_face_attribute<9>(name, [&field](size_t i) {
                Eigen::Matrix<double, 9, 1> v;
                for (int k = 0; k < 9; ++k) v[k] = field[i][k];
                return v;
            });
        } else {
            logger().warn("{}: field {} has {} components per face; not written", path, name, n);
        }
    }
    const auto& triangles = msh.m_spec.elements.entity_blocks.back();
    for (const auto& [name, field] : s.corner_fields) {
        if (field.empty() || field.front().empty() || field.front().size() % 3 != 0) {
            logger().warn(
                "{}: per-corner field {} is empty or not 3 corners; not written",
                path,
                name);
            continue;
        }
        mshio::Data data;
        data.header.string_tags = {name};
        data.header.real_tags = {0.0};
        data.header.int_tags = {0, int(field.front().size() / 3), int(s.faces.size()), 0};
        data.entries.resize(s.faces.size());
        for (size_t i = 0; i < s.faces.size(); ++i) {
            data.entries[i].tag = triangles.data[i * 4]; // tag, then the three nodes
            data.entries[i].num_nodes_per_element = 3;
            data.entries[i].data = field[i];
        }
        msh.m_spec.element_node_data.push_back(std::move(data));
    }
    msh.save(path);
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
