#pragma once

// Drop-in replacements for the small subset of Intel TBB that wildmeshing-toolkit
// uses, implemented on top of the C++ standard library (std::thread & friends).
//
// The goal is a *mechanical* migration off TBB: every `tbb::X` used in the code
// base has a `wmtk::X` counterpart here with the same (used) API surface, so the
// only edits required elsewhere are `#include <tbb/...>` -> this header and
// `tbb::` -> `wmtk::`.
//
// Notes / intentional simplifications:
//  * Containers that used to grow concurrently (attribute / connectivity storage)
//    are NOT provided here -- they became plain std::vector, preallocated up front
//    (see TetMesh/TriMesh). `collector` here only backs the few "collect
//    results from parallel loops" sites, and is a mutex-guarded deque so that
//    references to existing elements stay valid while other threads append and so
//    that bool elements are not bit-packed (concurrent distinct-index writes stay
//    safe).
//  * enumerable_thread_specific is lock-free on the hot path: each thread keeps a
//    small thread_local list of (instance-id -> value) slots.

#include "collector.hpp"
#include "concurrent_map.hpp"
#include "concurrent_priority_queue.hpp"
#include "concurrent_queue.hpp"
#include "enumerable_thread_specific.hpp"
#include "parallel_for.hpp"
#include "range.hpp"
#include "spin_mutex.hpp"
#include "task_group.hpp"