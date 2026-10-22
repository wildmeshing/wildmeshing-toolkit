#pragma once

#include <cstdint>

/// The history types by name, for headers that only hold them (History.hpp defines them).
namespace wmtk::history {

enum class EditKind : uint8_t;
struct Edit;

template <int N>
struct History;
template <int N>
class BasicRecorder;
template <int N>
class BasicReplay;
template <int N>
class TrackedRecorder;

/// A surface: faces are triangles.
using SurfaceHistory = History<3>;
/// A curve network: faces are segments.
using CurveHistory = History<2>;
using Recorder = BasicRecorder<3>;
using CurveRecorder = BasicRecorder<2>;
using Replay = BasicReplay<3>;
using CurveReplay = BasicReplay<2>;
using TrackedSurfaceRecorder = TrackedRecorder<3>;
using TrackedCurveRecorder = TrackedRecorder<2>;

} // namespace wmtk::history
