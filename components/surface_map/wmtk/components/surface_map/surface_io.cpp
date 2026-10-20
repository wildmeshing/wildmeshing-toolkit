#include "surface_io.hpp"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>

#include <filesystem>
#include <fstream>

namespace wmtk::components::surface_map {

SurfaceData read_surface_msh(const std::string& path)
{
    MshData msh;
    try {
        msh.load(path);
    } catch (const std::exception& e) {
        log_and_throw_error("Could not read {}: {}", path, e.what());
    }
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
            field[i] = data;
        });
    }
    const mshio::ElementBlock* triangles = nullptr;
    for (const auto& block : msh.m_spec.elements.entity_blocks) {
        if (block.entity_dim == 2) triangles = &block;
    }
    for (const mshio::Data& data : msh.m_spec.element_node_data) {
        if (data.header.string_tags.empty() || triangles == nullptr) continue;
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
