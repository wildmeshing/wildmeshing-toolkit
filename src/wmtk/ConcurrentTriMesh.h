#pragma once

#include <wmtk/TriMesh.h>
#include <tbb/concurrent_vector.h>
#include <tbb/spin_mutex.h>

namespace wmtk {
class ConcurrentTriMesh : public TriMesh
{
public:
    ConcurrentTriMesh() = default;
    virtual ~ConcurrentTriMesh() = default; 

};
} // namespace wmtk

