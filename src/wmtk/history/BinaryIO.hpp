#pragma once

// The volume history files' binary encoding, the surface history's (History.cpp): unsigned
// LEB128 varints, little-endian f64s.

#include <wmtk/utils/Logger.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace wmtk::history::detail {

/// n as an unsigned LEB128 varint, a byte at a time to `put`.
template <typename Put>
void put_varint(uint64_t n, Put&& put)
{
    do {
        uint8_t b = n & 0x7F;
        n >>= 7;
        if (n) b |= 0x80;
        put(b);
    } while (n);
}

/// A varint from the bytes `get` returns, into n; false if it runs past 64 bits.
template <typename Get>
bool get_varint(Get&& get, uint64_t& n)
{
    n = 0;
    for (int shift = 0; shift < 64; shift += 7) {
        const uint8_t b = get();
        n |= uint64_t(b & 0x7F) << shift;
        if (!(b & 0x80)) return true;
    }
    return false;
}

/// A signed integer as an unsigned one that is small when its magnitude is: 0, -1, 1, -2, ...
/// become 0, 1, 2, 3, ...
inline uint64_t zigzag(const int64_t x)
{
    return (uint64_t(x) << 1) ^ uint64_t(x >> 63);
}

inline int64_t unzigzag(const uint64_t x)
{
    return int64_t(x >> 1) ^ -int64_t(x & 1);
}

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
        put_varint(n, [this](const uint8_t b) { put(b); });
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
        uint64_t n;
        if (!get_varint([this]() { return get<uint8_t>(); }, n)) {
            log_and_throw_error("history: {} holds a malformed integer", m_path);
        }
        return n;
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

/// Writer's encoding, appended to a byte buffer: for a history that encodes in memory as it
/// records (VolumeRecorder).
class BufferWriter
{
public:
    explicit BufferWriter(std::vector<uint8_t>& out)
        : m_out(out)
    {}
    void byte(const uint8_t b) { m_out.push_back(b); }
    void varint(const uint64_t n)
    {
        put_varint(n, [this](const uint8_t b) { m_out.push_back(b); });
    }
    void position(const Eigen::Vector3d& p)
    {
        const size_t at = m_out.size();
        m_out.resize(at + 3 * sizeof(double));
        std::memcpy(m_out.data() + at, p.data(), 3 * sizeof(double));
    }

private:
    std::vector<uint8_t>& m_out;
};

/// Reader's decoding, from a byte buffer [p, end): throws "<what>: <name> is corrupt" when a
/// read runs past the end or a count could not fit in what is left.
class BufferReader
{
public:
    BufferReader(const uint8_t* p, const uint8_t* end, std::string what, std::string name)
        : m_p(p)
        , m_end(end)
        , m_what(std::move(what))
        , m_name(std::move(name))
    {}
    [[noreturn]] void corrupt() const { log_and_throw_error("{}: {} is corrupt", m_what, m_name); }
    /// The bytes not read yet.
    uint64_t remaining() const { return uint64_t(m_end - m_p); }
    const uint8_t* at() const { return m_p; }
    uint8_t byte()
    {
        if (m_p >= m_end) corrupt();
        return *m_p++;
    }
    uint64_t varint()
    {
        uint64_t n;
        if (!get_varint([this]() { return byte(); }, n)) corrupt();
        return n;
    }
    /// A count of items of at least `min_bytes` each, which the rest of the buffer must hold.
    uint64_t count(const uint64_t min_bytes = 1)
    {
        const uint64_t n = varint();
        if (n > remaining() / min_bytes) corrupt();
        return n;
    }
    template <typename T>
    T get()
    {
        if (remaining() < sizeof(T)) corrupt();
        T x;
        std::memcpy(&x, m_p, sizeof(T));
        m_p += sizeof(T);
        return x;
    }
    Eigen::Vector3d position()
    {
        Eigen::Vector3d x;
        for (int k = 0; k < 3; ++k) x[k] = get<double>();
        return x;
    }
    std::string string(const uint64_t n)
    {
        if (remaining() < n) corrupt();
        std::string s(reinterpret_cast<const char*>(m_p), size_t(n));
        m_p += n;
        return s;
    }

private:
    const uint8_t* m_p;
    const uint8_t* m_end;
    std::string m_what, m_name;
};

} // namespace wmtk::history::detail
