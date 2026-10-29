#pragma once

// The binary encoding the history files share -- the surface's and curves' (History.cpp) and the
// volume's (VolumeHistory.cpp): unsigned LEB128 varints, little-endian f64s, written to bytes in
// memory and read from them, which a file holds as they are.

#include <wmtk/utils/Logger.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <cstring>
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

/// Appends to the bytes of a history, as a file holds them.
class Writer
{
public:
    explicit Writer(std::vector<uint8_t>& out)
        : m_out(out)
    {}
    template <typename T>
    void put(const T& x)
    {
        const auto* p = reinterpret_cast<const uint8_t*>(&x);
        m_out.insert(m_out.end(), p, p + sizeof(T));
    }
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
    void bytes(const char* p, const size_t n)
    {
        m_out.insert(
            m_out.end(),
            reinterpret_cast<const uint8_t*>(p),
            reinterpret_cast<const uint8_t*>(p) + n);
    }

private:
    std::vector<uint8_t>& m_out;
};

/**
 * @brief Reads the bytes of a history, [p, end). `what` and `name` say what they are in messages,
 * "<what>: <name> is ...": the kind of history, and a file or the bytes given. A read past the end
 * throws "is truncated", a count that could not fit in what is left "is corrupt".
 */
class Reader
{
public:
    Reader(const uint8_t* p, const uint8_t* end, std::string what, std::string name)
        : m_p(p)
        , m_end(end)
        , m_what(std::move(what))
        , m_name(std::move(name))
    {}
    /// All of `in`, the bytes of a surface's or curves' history.
    Reader(const std::vector<uint8_t>& in, std::string name)
        : Reader(in.data(), in.data() + in.size(), "history", std::move(name))
    {}
    /// The bytes not read yet, and where they start.
    uint64_t remaining() const { return uint64_t(m_end - m_p); }
    const uint8_t* at() const { return m_p; }
    /// A count of items of at least `min_bytes` each, which the rest of the bytes must hold: a
    /// damaged count cannot make the reader allocate more than the bytes could fill.
    uint64_t count(const uint64_t min_bytes = 1)
    {
        const uint64_t n = varint();
        if (n > remaining() / min_bytes) corrupt();
        return n;
    }
    [[noreturn]] void corrupt() const { log_and_throw_error("{}: {} is corrupt", m_what, m_name); }
    template <typename T>
    T get()
    {
        T x;
        bytes(reinterpret_cast<char*>(&x), sizeof(T));
        return x;
    }
    uint8_t byte() { return get<uint8_t>(); }
    uint64_t varint()
    {
        uint64_t n;
        if (!get_varint([this]() { return byte(); }, n)) {
            log_and_throw_error("{}: {} holds a malformed integer", m_what, m_name);
        }
        return n;
    }
    Eigen::Vector3d position()
    {
        Eigen::Vector3d x;
        for (int k = 0; k < 3; ++k) x[k] = get<double>();
        return x;
    }
    void bytes(char* p, const size_t n)
    {
        if (n > remaining()) log_and_throw_error("{}: {} is truncated", m_what, m_name);
        std::memcpy(p, m_p, n);
        m_p += n;
    }
    std::string string(const uint64_t n)
    {
        if (n > remaining()) log_and_throw_error("{}: {} is truncated", m_what, m_name);
        std::string s(reinterpret_cast<const char*>(m_p), size_t(n));
        m_p += n;
        return s;
    }
    const std::string& name() const { return m_name; }

private:
    const uint8_t* m_p;
    const uint8_t* m_end;
    std::string m_what, m_name;
};

} // namespace wmtk::history::detail
