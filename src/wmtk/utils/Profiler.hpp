#pragma once

/**
 * @file Profiler.hpp
 * @brief Run-time and mesh-size profiling, written to one JSON file (profiling branch only).
 *
 * Two kinds of record, both off (one relaxed atomic load each) unless a Session is open:
 *
 *  - SCOPES (Scope): named steps of the algorithm, nested, recorded on the thread that opened the
 *    Session only (the driver's thread; the passes are sequential there). Each holds its wall
 *    time (the profiler's own bookkeeping subtracted), the mesh size at its start and end (the
 *    registered SizeProbe; `sizes` false skips it), the resident memory at its end, numeric
 *    attributes (turn, pass index, executor statistics, ...) and the change of every call counter
 *    over the scope. A Scope opened on another thread does nothing.
 *
 *  - CALL COUNTERS (WMTK_PROF_CALL): calls and summed time of a function, from every thread, kept
 *    per thread and added up when read. The time is THREAD time: under N threads the counters of
 *    a pass can add up to N times its wall time. Counters nest (a counter inside another's call
 *    is in both), so the sum over counters is not a total.
 *
 * The JSON (Session::write()) holds the scopes as a flat list with parent indices, each with its
 * path of names from the root, the counters' totals, and per-path totals of the scopes.
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace wmtk::prof {

/// True while a Session is open.
extern std::atomic<bool> g_enabled;
inline bool enabled()
{
    return g_enabled.load(std::memory_order_relaxed);
}

inline uint64_t now_ns()
{
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count());
}

/// How many distinct counters a run can have; a name past it is counted under "<overflow>".
constexpr int kMaxCounters = 512;

/// The id of the counter `name`, registered on first use (thread-safe, idempotent).
int counter_id(const std::string& name);
/// One call of `ns` nanoseconds on the calling thread's counter `id`.
void counter_add(int id, uint64_t ns);
/// `n` events without a time (calls only).
void counter_count(int id, uint64_t n = 1);

/// Times its own lifetime into counter `id` (when profiling is on).
class CallTimer
{
public:
    explicit CallTimer(const int id)
        : m_id(enabled() ? id : -1)
        , m_t0(m_id >= 0 ? now_ns() : 0)
    {}
    ~CallTimer()
    {
        if (m_id >= 0) counter_add(m_id, now_ns() - m_t0);
    }
    /// Bill the call to another counter (e.g. an operation's "ok" or "fail" counter).
    void retarget(const int id)
    {
        if (m_id >= 0) m_id = id;
    }
    CallTimer(const CallTimer&) = delete;
    CallTimer& operator=(const CallTimer&) = delete;

private:
    int m_id;
    uint64_t m_t0;
};

/// Mesh size at a scope's start and end; -1 is "not measured".
struct MeshSize
{
    int64_t V = -1; ///< live vertices
    int64_t T = -1; ///< live cells (tets, or triangles in 2D)
    int64_t band_T = -1; ///< cells of the offset band
    int64_t front_V = -1; ///< vertices on the offset surface
};
using SizeProbe = std::function<MeshSize()>;

/// The probe the scopes read (owner thread). Cleared by the guard's destructor, so it never
/// outlives the mesh it reads.
class ProbeGuard
{
public:
    explicit ProbeGuard(SizeProbe probe);
    ~ProbeGuard();
    ProbeGuard(const ProbeGuard&) = delete;
    ProbeGuard& operator=(const ProbeGuard&) = delete;

private:
    bool m_set = false;
};

/// One step of the algorithm. Ends at end() or at its destruction, whichever comes first.
class Scope
{
public:
    explicit Scope(const char* name, bool sizes = true);
    explicit Scope(const std::string& name, bool sizes = true);
    ~Scope() { end(); }
    /// A numeric attribute (overwrites).
    Scope& attr(const char* key, double value);
    /// A text attribute (overwrites).
    Scope& tag(const char* key, const std::string& value);
    void end();
    bool active() const { return m_event >= 0; }

    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;

private:
    int m_event = -1; ///< the event's index; -1: not recording
    uint64_t m_gen = 0; ///< the session it was opened in
};

/// Attributes of the innermost open scope (owner thread; ignored elsewhere).
void attr(const char* key, double value);
void attr_add(const char* key, double value);
void tag(const char* key, const std::string& value);

/// One call of an ExecutePass (a pass's executor run), added up into the innermost open scope's
/// "exec.*" attributes.
struct ExecStats
{
    double attempts = 0, successes = 0, failures = 0;
    double lock_failures = 0, overflowed = 0, final_queue = 0;
    double parallel_seconds = 0, serial_tail_seconds = 0;
    double busiest_task_seconds = 0, idlest_task_seconds = 0;
    double rounds = 1;
    bool screened = false;
    double screen_seconds = 0, commit_seconds = 0;
};
void executor_pass(const ExecStats& s);

/**
 * @brief A profiling run: open from construction to destruction, written at destruction (also
 * when an exception unwinds through it, marked "status": "exception"). An empty path does
 * nothing. One Session at a time.
 */
class Session
{
public:
    explicit Session(const std::string& path, const std::string& root_name = "run");
    ~Session();
    bool active() const { return m_active; }
    /// Metadata, as a JSON text (parsed when written) or a number.
    void meta_json(const std::string& key, const std::string& json_text);
    void meta(const std::string& key, const std::string& value);
    void meta(const std::string& key, double value);

    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

private:
    bool m_active = false;
    int m_uncaught = 0;
};

/// Write the JSON now, with the scopes still open marked "open" and timed to now: a run killed
/// later still leaves the profile up to here. No-op without a Session.
void checkpoint();

} // namespace wmtk::prof

#define WMTK_PROF_CAT2_(a, b) a##b
#define WMTK_PROF_CAT_(a, b) WMTK_PROF_CAT2_(a, b)

/// Times the rest of the enclosing block into the call counter `name` (a string literal).
#define WMTK_PROF_CALL(name)                                                         \
    static const int WMTK_PROF_CAT_(wmtk_prof_id_, __LINE__) =                       \
        ::wmtk::prof::counter_id(name);                                              \
    const ::wmtk::prof::CallTimer WMTK_PROF_CAT_(wmtk_prof_timer_, __LINE__)(        \
        WMTK_PROF_CAT_(wmtk_prof_id_, __LINE__))

/// The value of `expr`, its evaluation timed into the call counter `name`.
#define WMTK_PROF_TIMED(name, expr) \
    ([&]() {                        \
        WMTK_PROF_CALL(name);       \
        return (expr);              \
    }())

/// A step of the algorithm as a scope on the rest of the enclosing block.
#define WMTK_PROF_SCOPE(name) \
    ::wmtk::prof::Scope WMTK_PROF_CAT_(wmtk_prof_scope_, __LINE__)(name)
/// As WMTK_PROF_SCOPE, without the mesh size (for steps that do not change the mesh).
#define WMTK_PROF_SCOPE_NOSIZE(name) \
    ::wmtk::prof::Scope WMTK_PROF_CAT_(wmtk_prof_scope_, __LINE__)(name, false)
