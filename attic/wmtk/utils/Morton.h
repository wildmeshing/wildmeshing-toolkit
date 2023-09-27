/*
    This file is part of NSEssentials.

    Use of this source code is granted via a BSD-style license, which can be found
    in License.txt in the repository root.

    @author Nico Schertler
*/

#pragma once
#ifdef NSE_BUILD_SHARED
#ifndef NSE_EXPORT
#ifdef _WIN32
#ifdef nsessentials_EXPORTS
/* We are building this library */
#define NSE_EXPORT __declspec(dllexport)
#else
/* We are using this library */
#define NSE_EXPORT __declspec(dllimport)
#endif
#else
#define NSE_EXPORT
#endif
#endif
#else
#define NSE_EXPORT
#endif
#include <stdint.h>
#include <cinttypes>

namespace Resorting {

// Represents a three-dimensional 64-bit Morton Code.
class NSE_EXPORT MortonCode64
{
public:
    MortonCode64();

    MortonCode64(int32_t x, int32_t y, int32_t z);

    MortonCode64(uint32_t x, uint32_t y, uint32_t z);

    MortonCode64(uint64_t);

    // Decodes the code into its three coordinates.
    void decode(int32_t& x, int32_t& y, int32_t& z) const;

    // Negates the specified coordinate.
    template <int DIM>
    MortonCode64 InvertDimension() const;

    // Under the assumption that all entries are positive
    MortonCode64 DivideDimensionBy2(int dim) const;
    MortonCode64 Negate() const;

    MortonCode64 operator+(const MortonCode64 rhs) const;
    MortonCode64 operator+(const int64_t rhs) const;
    MortonCode64& operator+=(const MortonCode64 rhs);
    MortonCode64 operator-(const MortonCode64 rhs) const;

    // Applies the bitshift to every dimension, assumes all entries to be positive
    MortonCode64 operator>>(int shift) const;
    MortonCode64 operator<<(int shift) const;

    bool operator<(const MortonCode64 rhs) const { return data < rhs.data; }
    bool operator>(const MortonCode64 rhs) const { return data > rhs.data; }
    bool operator<=(const MortonCode64 rhs) const { return data <= rhs.data; }
    bool operator>=(const MortonCode64 rhs) const { return data >= rhs.data; }
    bool operator==(const MortonCode64 rhs) const { return data == rhs.data; }
    bool operator!=(const MortonCode64 rhs) const { return data != rhs.data; }

    explicit operator uint64_t() const { return data; }

    static const MortonCode64 Zero;
    static const MortonCode64 UnitX;
    static const MortonCode64 UnitY;
    static const MortonCode64 UnitZ;

private:
    uint64_t data;
};

} // namespace Resorting