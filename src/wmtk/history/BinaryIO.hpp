#pragma once

// The volume history files' binary encoding, the surface history's (History.cpp): unsigned
// LEB128 varints, little-endian f64s.

#include <wmtk/utils/Logger.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <fstream>
#include <string>

namespace wmtk::history::detail {

class Writer
{
public:
    explicit Writer(const std::string& path)
        : m_out(path, std::ios::binary)
        , m_path(path)
    {
        if (!m_out) log_and_throw_error("history: cannot write {}", path);
    }
    template <typename T>
    void put(const T& x)
    {
        m_out.write(reinterpret_cast<const char*>(&x), sizeof(T));
    }
    void varint(uint64_t n)
    {
        do {
            uint8_t b = n & 0x7F;
            n >>= 7;
            if (n) b |= 0x80;
            put(b);
        } while (n);
    }
    void position(const Eigen::Vector3d& p)
    {
        for (int k = 0; k < 3; ++k) put(p[k]);
    }
    void bytes(const char* p, size_t n) { m_out.write(p, std::streamsize(n)); }
    void finish()
    {
        m_out.flush();
        if (!m_out) log_and_throw_error("history: writing {} failed", m_path);
    }

private:
    std::ofstream m_out;
    std::string m_path;
};

class Reader
{
public:
    explicit Reader(const std::string& path)
        : m_in(path, std::ios::binary)
        , m_path(path)
    {
        if (!m_in) log_and_throw_error("history: cannot read {}", path);
        m_in.seekg(0, std::ios::end);
        m_size = uint64_t(m_in.tellg());
        m_in.seekg(0);
    }
    /// The bytes not read yet.
    uint64_t remaining() { return m_size - uint64_t(m_in.tellg()); }
    /// A count of items of at least `min_bytes` each, which the rest of the file must hold: a
    /// damaged count cannot make the reader allocate more than the file could fill.
    uint64_t count(const uint64_t min_bytes = 1)
    {
        const uint64_t n = varint();
        if (n > remaining() / min_bytes) corrupt();
        return n;
    }
    [[noreturn]] void corrupt() const { log_and_throw_error("history: {} is corrupt", m_path); }
    template <typename T>
    T get()
    {
        T x;
        m_in.read(reinterpret_cast<char*>(&x), sizeof(T));
        if (!m_in) log_and_throw_error("history: {} is truncated", m_path);
        return x;
    }
    uint64_t varint()
    {
        uint64_t n = 0;
        for (int shift = 0; shift < 64; shift += 7) {
            const uint8_t b = get<uint8_t>();
            n |= uint64_t(b & 0x7F) << shift;
            if (!(b & 0x80)) return n;
        }
        log_and_throw_error("history: {} holds a malformed integer", m_path);
    }
    Eigen::Vector3d position()
    {
        Eigen::Vector3d p;
        for (int k = 0; k < 3; ++k) p[k] = get<double>();
        return p;
    }
    void bytes(char* p, size_t n)
    {
        m_in.read(p, std::streamsize(n));
        if (!m_in) log_and_throw_error("history: {} is truncated", m_path);
    }

private:
    std::ifstream m_in;
    std::string m_path;
    uint64_t m_size = 0;
};

} // namespace wmtk::history::detail
