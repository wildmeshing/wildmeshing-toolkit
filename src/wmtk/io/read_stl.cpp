// Derived from libigl's igl::readSTL -- https://github.com/libigl/libigl, include/igl/readSTL.cpp
//
// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Alec Jacobson <alecjacobson@gmail.com>
// Copyright (C) 2018 Qingnan Zhou <qnzhou@gmail.com>
// Copyright (C) 2020 Jérémie Dumas <jeremie.dumas@ens-lyon.org>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//
// Modified for the wildmeshing toolkit. What changed and why:
//
//  * Every assert() is now an error naming the file. The originals abort a Debug build
//    from inside the reader -- an assert cannot be caught, so no amount of care at the
//    call site helps. A malformed mesh in a batch has to be reportable, not fatal.
//  * Facet data is validated before it is used. The original reads into a buffer, casts
//    the buffer to floats, and only then asks whether the read succeeded, so a short read
//    is interpreted as whatever the buffer happened to still hold.
//  * A binary STL's declared facet count is checked against the file length up front, so
//    truncation is one precise error instead of a failure part-way through parsing.
//  * Format detection uses that length invariant as the primary signal rather than the
//    "solid" prefix, which binary writers also emit (the two files that motivated this
//    begin "COLOR=" and "Created by ViaCAD Pro").
//  * Empty input yields 0x3 matrices, not 0x0. libigl returns 0x0, which silently breaks
//    every downstream assumption of three columns -- including Eigen colwise() reductions,
//    which dereference the null data pointer of an empty matrix rather than short-circuit.
//  * Floats are loaded with memcpy rather than reinterpret_cast, which was an aliasing and
//    alignment violation.
//  * An ASCII facet must have exactly three vertices. The original warns and returns false.
//  * Normals are discarded and the templates are gone; the toolkit needs neither.

#include "read_stl.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace wmtk::io {

namespace {

constexpr std::streamsize HEADER_SIZE = 80;
constexpr std::streamsize COUNT_SIZE = 4;
constexpr std::streamsize PREAMBLE_SIZE = HEADER_SIZE + COUNT_SIZE;
constexpr std::streamsize FACET_SIZE = 50; // 12 floats then a 2-byte attribute word
constexpr std::streamsize NORMAL_SIZE = 12;
constexpr std::streamsize VERTEX_SIZE = 12;

/// `buf` carries no alignment guarantee, so this cannot be a reinterpret_cast.
float load_float(const char* buf)
{
    float x = 0.f;
    std::memcpy(&x, buf, sizeof(x));
    return x;
}

std::string lowercase(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

/// An STL shares no vertices between facets, so #V is 3 * #F and the faces are just
/// consecutive triples.
void fill_faces(std::int64_t n_facets, MatrixXi& F)
{
    F.resize(n_facets, 3);
    for (std::int64_t i = 0; i < n_facets; ++i) {
        F.row(i) << static_cast<int>(3 * i), static_cast<int>(3 * i + 1),
            static_cast<int>(3 * i + 2);
    }
}

void read_binary(
    std::ifstream& in,
    const fs::path& path,
    std::uintmax_t file_size,
    std::uint32_t n_facets,
    MatrixXd& V,
    MatrixXi& F)
{
    const std::uintmax_t expected = static_cast<std::uintmax_t>(PREAMBLE_SIZE) +
                                    static_cast<std::uintmax_t>(FACET_SIZE) * n_facets;
    if (file_size != expected) {
        log_and_throw_error(
            "Could not read STL {}: the header declares {} facets, so the file should be "
            "{} bytes, but it is {} ({} short)",
            path.string(),
            n_facets,
            expected,
            file_size,
            expected > file_size ? expected - file_size : 0);
    }

    V.resize(3 * static_cast<std::int64_t>(n_facets), 3);
    in.clear();
    in.seekg(PREAMBLE_SIZE);

    std::array<char, static_cast<std::size_t>(FACET_SIZE)> facet{};
    for (std::uint32_t i = 0; i < n_facets; ++i) {
        in.read(facet.data(), FACET_SIZE);
        if (in.gcount() != FACET_SIZE) {
            log_and_throw_error(
                "Could not read STL {}: the file ends part-way through facet {} of {}",
                path.string(),
                i,
                n_facets);
        }

        for (int v = 0; v < 3; ++v) {
            const char* p = facet.data() + NORMAL_SIZE + VERTEX_SIZE * v;
            const double x = load_float(p);
            const double y = load_float(p + 4);
            const double z = load_float(p + 8);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                log_and_throw_error(
                    "Could not read STL {}: facet {} has a NaN or infinite coordinate",
                    path.string(),
                    i);
            }
            V.row(3 * static_cast<std::int64_t>(i) + v) << x, y, z;
        }
    }

    fill_faces(static_cast<std::int64_t>(n_facets), F);
}

void expect_token(std::ifstream& in, const fs::path& path, const char* expected)
{
    std::string token;
    if (!(in >> token)) {
        log_and_throw_error(
            "Could not read STL {}: the file ends where '{}' was expected",
            path.string(),
            expected);
    }
    if (lowercase(token) != expected) {
        log_and_throw_error(
            "Could not read STL {}: expected '{}', found '{}'",
            path.string(),
            expected,
            token);
    }
}

double parse_number(std::ifstream& in, const fs::path& path, const char* context)
{
    double x = 0;
    if (!(in >> x)) {
        log_and_throw_error(
            "Could not read STL {}: expected a number for {}",
            path.string(),
            context);
    }
    if (!std::isfinite(x)) {
        log_and_throw_error("Could not read STL {}: {} is NaN or infinite", path.string(), context);
    }
    return x;
}

void read_ascii(std::ifstream& in, const fs::path& path, MatrixXd& V, MatrixXi& F)
{
    in.clear();
    in.seekg(0);

    std::vector<std::array<double, 3>> verts;
    std::string token;
    bool saw_solid = false;

    while (in >> token) {
        const std::string keyword = lowercase(token);

        if (keyword == "solid") {
            saw_solid = true;
            std::getline(in, token); // the solid's name, which carries no meaning
            continue;
        }
        if (keyword == "endsolid") {
            // Not a stopping point: one file may hold several solids, and the facets of
            // all of them belong to the mesh. Thingi10K 1080515.stl has four, so stopping
            // at the first would silently return a quarter of the model.
            std::getline(in, token);
            continue;
        }
        if (keyword != "facet") {
            log_and_throw_error(
                "Could not read STL {}: expected 'facet', 'solid' or 'endsolid' after "
                "facet {}, found '{}'",
                path.string(),
                verts.size() / 3,
                token);
        }

        expect_token(in, path, "normal");
        // Skipped as three opaque tokens rather than parsed as numbers. Normals are
        // discarded, and real files carry unparseable ones -- a good few Thingi10K meshes
        // say "facet normal NaN NaN NaN" over otherwise perfectly good geometry. Demanding
        // a number here rejects a mesh over a field nothing reads.
        for (int i = 0; i < 3; ++i) {
            if (!(in >> token)) {
                log_and_throw_error(
                    "Could not read STL {}: the file ends inside the normal of facet {}",
                    path.string(),
                    verts.size() / 3);
            }
        }

        expect_token(in, path, "outer");
        expect_token(in, path, "loop");
        // Exactly three, not "however many turn up": a facet with any other count is not a
        // triangle, and silently accepting one corrupts the face indices for every facet
        // after it.
        for (int v = 0; v < 3; ++v) {
            expect_token(in, path, "vertex");
            const double x = parse_number(in, path, "a vertex x coordinate");
            const double y = parse_number(in, path, "a vertex y coordinate");
            const double z = parse_number(in, path, "a vertex z coordinate");
            verts.push_back({{x, y, z}});
        }
        expect_token(in, path, "endloop");
        expect_token(in, path, "endfacet");
    }

    if (!saw_solid) {
        log_and_throw_error(
            "Could not read STL {}: it is neither a well-formed binary STL nor an ASCII "
            "one (no 'solid' keyword)",
            path.string());
    }

    V.resize(static_cast<std::int64_t>(verts.size()), 3);
    for (std::size_t i = 0; i < verts.size(); ++i) {
        V.row(static_cast<std::int64_t>(i)) << verts[i][0], verts[i][1], verts[i][2];
    }
    fill_faces(static_cast<std::int64_t>(verts.size() / 3), F);
}

} // namespace

void read_stl(const fs::path& path, MatrixXd& V, MatrixXi& F)
{
    // Set the shape up front so that every early return, the empty mesh included, hands
    // back three columns.
    V.resize(0, 3);
    F.resize(0, 3);

    std::error_code ec;
    const std::uintmax_t file_size = fs::file_size(path, ec);
    if (ec) {
        log_and_throw_error("Could not read STL {}: {}", path.string(), ec.message());
    }

    std::ifstream in(path, std::ios::binary);
    if (!in) {
        log_and_throw_error("Could not open STL {}", path.string());
    }

    std::array<char, static_cast<std::size_t>(PREAMBLE_SIZE)> preamble{};
    in.read(preamble.data(), PREAMBLE_SIZE);
    const bool has_preamble = in.gcount() == PREAMBLE_SIZE;

    if (has_preamble) {
        std::uint32_t n_facets = 0;
        std::memcpy(&n_facets, preamble.data() + HEADER_SIZE, sizeof(n_facets));
        const std::uintmax_t expected = static_cast<std::uintmax_t>(PREAMBLE_SIZE) +
                                        static_cast<std::uintmax_t>(FACET_SIZE) * n_facets;

        // The length invariant is the only self-consistent signal either format offers, so
        // it decides first. The "solid" prefix does not: binary writers emit it too, and
        // the files that motivated this reader begin "COLOR=" and "Created by ViaCAD Pro".
        if (file_size == expected) {
            read_binary(in, path, file_size, n_facets, V, F);
            return;
        }

        if (lowercase(std::string(preamble.data(), 5)) != "solid") {
            // It claims to be binary and the length disagrees, and it cannot be ASCII
            // either. read_binary reports the discrepancy, which is the actionable part.
            read_binary(in, path, file_size, n_facets, V, F);
            return;
        }
    }

    read_ascii(in, path, V, F);
}

} // namespace wmtk::io
