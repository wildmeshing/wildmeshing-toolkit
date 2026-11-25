#include "Profiler.hpp"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/getRSS.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <ctime>
#if !defined(_WIN32)
#include <unistd.h>
#endif

namespace wmtk::prof {

std::atomic<bool> g_enabled{false};

namespace {

struct Slot
{
    std::atomic<uint64_t> calls{0};
    std::atomic<uint64_t> ns{0};
    std::atomic<uint64_t> max_ns{0};
};

/// One thread's counters. Written by that thread only (load + store, no read-modify-write
/// contention); read by the owner thread at scope boundaries and when the JSON is written.
struct ThreadSlots
{
    std::array<Slot, kMaxCounters> s;
};

struct Totals
{
    uint64_t calls = 0;
    uint64_t ns = 0;
    uint64_t max_ns = 0;
};

struct Event
{
    std::string name;
    int parent = -1;
    int depth = 0;
    bool open = true;
    bool sizes = false;
    uint64_t t_begin = 0; ///< after the scope's own start bookkeeping
    uint64_t ovh_begin = 0; ///< the owner's bookkeeping total at t_begin
    double start_s = 0.; ///< since the session's start
    double seconds = 0.; ///< wall time, the profiler's bookkeeping inside excluded
    MeshSize size_begin, size_end;
    double rss_begin_mb = -1., rss_end_mb = -1.;
    std::vector<std::pair<std::string, double>> attrs;
    std::vector<std::pair<std::string, std::string>> tags;
    std::vector<Totals> counters_begin; ///< released when the scope ends
    std::vector<std::pair<int, Totals>> counter_delta;
};

struct State
{
    std::mutex mutex; ///< counter names and the thread list
    std::vector<std::string> names;
    std::unordered_map<std::string, int> ids;
    std::deque<std::unique_ptr<ThreadSlots>> threads;

    // The session; touched by the owner thread only (owner itself is read anywhere).
    std::atomic<std::thread::id> owner{};
    bool active = false;
    uint64_t generation = 0;
    std::string path;
    std::string root_name;
    uint64_t t0 = 0;
    uint64_t overhead = 0;
    std::vector<Event> events;
    std::vector<int> stack;
    SizeProbe probe;
    std::vector<std::pair<std::string, std::string>> meta_json;
    double timer_pair_ns = 0.;
    int checkpoints = 0;
    int root = -1;
};

State& st()
{
    static State s;
    return s;
}

ThreadSlots& my_slots()
{
    thread_local ThreadSlots* p = nullptr;
    if (p == nullptr) {
        State& s = st();
        std::lock_guard<std::mutex> lock(s.mutex);
        s.threads.push_back(std::make_unique<ThreadSlots>());
        p = s.threads.back().get();
    }
    return *p;
}

bool on_owner()
{
    return std::this_thread::get_id() == st().owner.load(std::memory_order_relaxed);
}

std::vector<Totals> snapshot()
{
    State& s = st();
    std::lock_guard<std::mutex> lock(s.mutex);
    const size_t n = s.names.size();
    std::vector<Totals> out(n);
    for (const auto& t : s.threads) {
        for (size_t i = 0; i < n; ++i) {
            const Slot& x = t->s[i];
            out[i].calls += x.calls.load(std::memory_order_relaxed);
            out[i].ns += x.ns.load(std::memory_order_relaxed);
            out[i].max_ns = std::max(out[i].max_ns, x.max_ns.load(std::memory_order_relaxed));
        }
    }
    return out;
}

void zero_all_slots()
{
    State& s = st();
    std::lock_guard<std::mutex> lock(s.mutex);
    for (const auto& t : s.threads) {
        for (Slot& x : t->s) {
            x.calls.store(0, std::memory_order_relaxed);
            x.ns.store(0, std::memory_order_relaxed);
            x.max_ns.store(0, std::memory_order_relaxed);
        }
    }
}

double mb(const size_t bytes)
{
    return double(bytes) / (1024. * 1024.);
}

std::vector<std::pair<int, Totals>> deltas(const std::vector<Totals>& a, const std::vector<Totals>& b)
{
    std::vector<std::pair<int, Totals>> out;
    for (size_t i = 0; i < b.size(); ++i) {
        const Totals z;
        const Totals& x = i < a.size() ? a[i] : z;
        if (b[i].calls == x.calls && b[i].ns == x.ns) continue;
        Totals d;
        d.calls = b[i].calls - x.calls;
        d.ns = b[i].ns - x.ns;
        out.emplace_back(int(i), d);
    }
    return out;
}

double own_seconds(const State& s, const Event& e, const uint64_t now)
{
    const int64_t d = int64_t(now - e.t_begin) - int64_t(s.overhead - e.ovh_begin);
    return d > 0 ? double(d) * 1e-9 : 0.;
}

void open_event(const std::string& name, const bool sizes, int& index, uint64_t& gen)
{
    State& s = st();
    const uint64_t b0 = now_ns();
    Event e;
    e.name = name;
    e.parent = s.stack.empty() ? -1 : s.stack.back();
    e.depth = int(s.stack.size());
    e.sizes = sizes && bool(s.probe);
    if (e.sizes) {
        e.size_begin = s.probe();
        e.rss_begin_mb = mb(getCurrentRSS());
    }
    e.counters_begin = snapshot();
    index = int(s.events.size());
    gen = s.generation;
    s.events.push_back(std::move(e));
    s.stack.push_back(index);
    const uint64_t b1 = now_ns();
    s.overhead += b1 - b0;
    Event& ev = s.events.back();
    ev.t_begin = b1;
    ev.ovh_begin = s.overhead;
    ev.start_s = double(b1 - s.t0) * 1e-9;
}

/// Ends the scope at the top of the stack, as of time `e0`, when the bookkeeping total was `ovh0`.
void close_top(const uint64_t e0, const uint64_t ovh0)
{
    State& s = st();
    const uint64_t c0 = now_ns();
    const int idx = s.stack.back();
    Event& e = s.events[size_t(idx)];
    const int64_t d = int64_t(e0 - e.t_begin) - int64_t(ovh0 - e.ovh_begin);
    e.seconds = d > 0 ? double(d) * 1e-9 : 0.;
    if (e.sizes && s.probe) {
        e.size_end = s.probe();
        e.rss_end_mb = mb(getCurrentRSS());
    }
    e.counter_delta = deltas(e.counters_begin, snapshot());
    std::vector<Totals>().swap(e.counters_begin);
    e.open = false;
    s.stack.pop_back();
    s.overhead += now_ns() - c0;
}

void close_event(const int idx, const uint64_t gen)
{
    State& s = st();
    if (!s.active || gen != s.generation) return;
    if (std::find(s.stack.begin(), s.stack.end(), idx) == s.stack.end()) return;
    const uint64_t e0 = now_ns();
    const uint64_t ovh0 = s.overhead;
    // A scope ended out of order ends the scopes opened inside it with it.
    while (!s.stack.empty()) {
        const int top = s.stack.back();
        close_top(e0, ovh0);
        if (top == idx) break;
    }
}

Event* top_event()
{
    if (!enabled() || !on_owner()) return nullptr;
    State& s = st();
    if (s.stack.empty()) return nullptr;
    return &s.events[size_t(s.stack.back())];
}

void set_attr(Event& e, const char* key, const double v, const bool add)
{
    for (auto& [k, x] : e.attrs) {
        if (k == key) {
            x = add ? x + v : v;
            return;
        }
    }
    e.attrs.emplace_back(key, v);
}

nlohmann::json size_json(const MeshSize& m)
{
    nlohmann::json j = nlohmann::json::object();
    if (m.V >= 0) j["V"] = m.V;
    if (m.T >= 0) j["T"] = m.T;
    if (m.band_T >= 0) j["band_T"] = m.band_T;
    if (m.front_V >= 0) j["front_V"] = m.front_V;
    return j;
}

struct Agg
{
    size_t count = 0;
    double seconds = 0., min_s = 0., max_s = 0.;
    void add(const double x)
    {
        min_s = count == 0 ? x : std::min(min_s, x);
        max_s = count == 0 ? x : std::max(max_s, x);
        seconds += x;
        ++count;
    }
    nlohmann::json json() const
    {
        return {
            {"count", count},
            {"seconds", seconds},
            {"min_seconds", min_s},
            {"max_seconds", max_s},
            {"mean_seconds", count ? seconds / double(count) : 0.}};
    }
};

void write_json(const std::string& status)
{
    State& s = st();
    const uint64_t now = now_ns();
    const std::vector<Totals> totals = snapshot();
    std::vector<std::string> names;
    {
        std::lock_guard<std::mutex> lock(s.mutex);
        names = s.names;
    }
    const auto counter_name = [&](const int i) {
        return size_t(i) < names.size() ? names[size_t(i)] : std::string("<unknown>");
    };

    nlohmann::json j;
    j["format"] = "wmtk_profile/1";
    j["status"] = status;
    j["checkpoints"] = s.checkpoints;
    j["wall_seconds"] = double(now - s.t0) * 1e-9;
    j["profiler_overhead_seconds"] = double(s.overhead) * 1e-9;
    j["timer_pair_ns"] = s.timer_pair_ns;
    j["peak_rss_mb"] = mb(getPeakRSS());
    j["notes"] = {
        "events[].seconds: wall time of the step, the profiler's own bookkeeping excluded; "
        "nested steps are inside their parent's time.",
        "events[].counters and counters: THREAD time summed over every thread (thread_seconds), "
        "so a parallel pass's counters can exceed its wall time; counters nest (a call timed "
        "inside another timed call is in both).",
        "size_start / size_end: V live vertices, T live tets, band_T band cells, front_V offset "
        "surface vertices, read at the step's start and end."};

    nlohmann::json meta = nlohmann::json::object();
    for (const auto& [k, text] : s.meta_json) {
        try {
            meta[k] = nlohmann::json::parse(text);
        } catch (const std::exception&) {
            meta[k] = text;
        }
    }
    j["meta"] = meta;

    // Paths from the root, names only.
    std::vector<std::string> path(s.events.size());
    for (size_t i = 0; i < s.events.size(); ++i) {
        const Event& e = s.events[i];
        path[i] = e.parent < 0 ? e.name : path[size_t(e.parent)] + "/" + e.name;
    }

    std::map<std::string, Agg> by_path, by_name;
    nlohmann::json events = nlohmann::json::array();
    for (size_t i = 0; i < s.events.size(); ++i) {
        const Event& e = s.events[i];
        const double secs = e.open ? own_seconds(s, e, now) : e.seconds;
        nlohmann::json ev;
        ev["id"] = i;
        ev["parent"] = e.parent;
        ev["depth"] = e.depth;
        ev["name"] = e.name;
        ev["path"] = path[i];
        ev["start"] = e.start_s;
        ev["seconds"] = secs;
        if (e.open) ev["open"] = true;
        if (e.sizes) {
            ev["size_start"] = size_json(e.size_begin);
            if (!e.open) ev["size_end"] = size_json(e.size_end);
            ev["rss_mb_start"] = e.rss_begin_mb;
            if (!e.open) ev["rss_mb_end"] = e.rss_end_mb;
        }
        if (!e.attrs.empty()) {
            nlohmann::json a = nlohmann::json::object();
            for (const auto& [k, v] : e.attrs) a[k] = v;
            ev["attrs"] = a;
        }
        if (!e.tags.empty()) {
            nlohmann::json a = nlohmann::json::object();
            for (const auto& [k, v] : e.tags) a[k] = v;
            ev["tags"] = a;
        }
        const auto d = e.open ? deltas(e.counters_begin, totals) : e.counter_delta;
        if (!d.empty()) {
            nlohmann::json c = nlohmann::json::object();
            for (const auto& [id, t] : d) {
                c[counter_name(id)] = {
                    {"calls", t.calls},
                    {"thread_seconds", double(t.ns) * 1e-9}};
            }
            ev["counters"] = c;
        }
        events.push_back(std::move(ev));
        if (!e.open) {
            by_path[path[i]].add(secs);
            by_name[e.name].add(secs);
        }
    }
    j["events"] = std::move(events);

    nlohmann::json counters = nlohmann::json::object();
    double est = 0.;
    for (size_t i = 0; i < totals.size(); ++i) {
        const Totals& t = totals[i];
        if (t.calls == 0) continue;
        est += double(t.calls) * s.timer_pair_ns * 1e-9;
        counters[counter_name(int(i))] = {
            {"calls", t.calls},
            {"thread_seconds", double(t.ns) * 1e-9},
            {"max_call_seconds", double(t.max_ns) * 1e-9},
            {"mean_call_us", double(t.ns) * 1e-3 / double(t.calls)}};
    }
    j["counters"] = std::move(counters);
    j["estimated_counter_overhead_thread_seconds"] = est;

    nlohmann::json bp = nlohmann::json::object(), bn = nlohmann::json::object();
    for (const auto& [k, a] : by_path) bp[k] = a.json();
    for (const auto& [k, a] : by_name) bn[k] = a.json();
    j["by_path"] = std::move(bp);
    j["by_name"] = std::move(bn);

    const std::filesystem::path out(s.path);
    if (out.has_parent_path()) std::filesystem::create_directories(out.parent_path());
    const std::filesystem::path tmp = out.string() + ".tmp";
    {
        std::ofstream f(tmp);
        if (!f) {
            logger().error("[profile] cannot write {}", tmp.string());
            return;
        }
        f << j.dump(1);
    }
    std::filesystem::rename(tmp, out);
}

void log_summary()
{
    State& s = st();
    const std::vector<Totals> totals = snapshot();
    std::vector<std::string> names;
    {
        std::lock_guard<std::mutex> lock(s.mutex);
        names = s.names;
    }
    std::vector<size_t> order;
    for (size_t i = 0; i < totals.size(); ++i) {
        if (totals[i].calls > 0) order.push_back(i);
    }
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return totals[a].ns > totals[b].ns;
    });
    logger().info(
        "[profile] {} steps, {} counters; the 12 counters with the most thread time:",
        s.events.size(),
        order.size());
    for (size_t k = 0; k < std::min<size_t>(12, order.size()); ++k) {
        const Totals& t = totals[order[k]];
        logger().info(
            "[profile]   {:<44} {:>12} calls {:>11.3f} thread-s {:>10.3f} us/call",
            names[order[k]],
            t.calls,
            double(t.ns) * 1e-9,
            double(t.ns) * 1e-3 / double(t.calls));
    }
}

} // namespace

int counter_id(const std::string& name)
{
    State& s = st();
    std::lock_guard<std::mutex> lock(s.mutex);
    if (const auto it = s.ids.find(name); it != s.ids.end()) return it->second;
    if (s.names.size() + 1 >= size_t(kMaxCounters)) {
        if (s.names.size() + 1 == size_t(kMaxCounters)) {
            s.names.emplace_back("<overflow>");
            s.ids["<overflow>"] = kMaxCounters - 1;
        }
        s.ids[name] = kMaxCounters - 1;
        return kMaxCounters - 1;
    }
    const int id = int(s.names.size());
    s.names.push_back(name);
    s.ids[name] = id;
    return id;
}

void counter_add(const int id, const uint64_t ns)
{
    if (id < 0 || id >= kMaxCounters) return;
    Slot& x = my_slots().s[size_t(id)];
    x.calls.store(x.calls.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    x.ns.store(x.ns.load(std::memory_order_relaxed) + ns, std::memory_order_relaxed);
    if (ns > x.max_ns.load(std::memory_order_relaxed)) {
        x.max_ns.store(ns, std::memory_order_relaxed);
    }
}

void counter_count(const int id, const uint64_t n)
{
    if (!enabled() || id < 0 || id >= kMaxCounters) return;
    Slot& x = my_slots().s[size_t(id)];
    x.calls.store(x.calls.load(std::memory_order_relaxed) + n, std::memory_order_relaxed);
}

ProbeGuard::ProbeGuard(SizeProbe probe)
{
    if (!enabled() || !on_owner()) return;
    st().probe = std::move(probe);
    m_set = true;
}

ProbeGuard::~ProbeGuard()
{
    if (m_set && on_owner()) st().probe = nullptr;
}

Scope::Scope(const char* name, const bool sizes)
{
    if (enabled() && on_owner() && st().active) open_event(std::string(name), sizes, m_event, m_gen);
}

Scope::Scope(const std::string& name, const bool sizes)
{
    if (enabled() && on_owner() && st().active) open_event(name, sizes, m_event, m_gen);
}

Scope& Scope::attr(const char* key, const double value)
{
    if (m_event >= 0 && st().active && size_t(m_event) < st().events.size()) {
        set_attr(st().events[size_t(m_event)], key, value, false);
    }
    return *this;
}

Scope& Scope::tag(const char* key, const std::string& value)
{
    if (m_event >= 0 && st().active && size_t(m_event) < st().events.size()) {
        auto& tags = st().events[size_t(m_event)].tags;
        for (auto& [k, v] : tags) {
            if (k == key) {
                v = value;
                return *this;
            }
        }
        tags.emplace_back(key, value);
    }
    return *this;
}

void Scope::end()
{
    if (m_event < 0) return;
    const int idx = m_event;
    m_event = -1;
    close_event(idx, m_gen);
}

void attr(const char* key, const double value)
{
    if (Event* e = top_event()) set_attr(*e, key, value, false);
}

void attr_add(const char* key, const double value)
{
    if (Event* e = top_event()) set_attr(*e, key, value, true);
}

void tag(const char* key, const std::string& value)
{
    if (Event* e = top_event()) {
        for (auto& [k, v] : e->tags) {
            if (k == key) {
                v = value;
                return;
            }
        }
        e->tags.emplace_back(key, value);
    }
}

void executor_pass(const ExecStats& x)
{
    Event* e = top_event();
    if (e == nullptr) return;
    set_attr(*e, "exec.calls", 1., true);
    set_attr(*e, "exec.attempts", x.attempts, true);
    set_attr(*e, "exec.successes", x.successes, true);
    set_attr(*e, "exec.failures", x.failures, true);
    set_attr(*e, "exec.lock_failures", x.lock_failures, true);
    set_attr(*e, "exec.overflowed", x.overflowed, true);
    set_attr(*e, "exec.final_queue", x.final_queue, true);
    set_attr(*e, "exec.parallel_seconds", x.parallel_seconds, true);
    set_attr(*e, "exec.serial_tail_seconds", x.serial_tail_seconds, true);
    set_attr(*e, "exec.busiest_task_seconds", x.busiest_task_seconds, true);
    set_attr(*e, "exec.idlest_task_seconds", x.idlest_task_seconds, true);
    double rounds = x.rounds;
    for (const auto& [k, v] : e->attrs) {
        if (k == "exec.max_rounds") rounds = std::max(rounds, v);
    }
    set_attr(*e, "exec.max_rounds", rounds, false);
    if (x.screened) {
        set_attr(*e, "exec.screened_calls", 1., true);
        set_attr(*e, "exec.screen_seconds", x.screen_seconds, true);
        set_attr(*e, "exec.commit_seconds", x.commit_seconds, true);
    }
}

Session::Session(const std::string& path, const std::string& root_name)
{
    if (path.empty()) return;
    State& s = st();
    if (s.active) {
        logger().warn("[profile] a profiling session is already open; {} is not written", path);
        return;
    }
    s.owner.store(std::this_thread::get_id(), std::memory_order_relaxed);
    s.path = path;
    s.root_name = root_name;
    s.events.clear();
    s.stack.clear();
    s.meta_json.clear();
    s.probe = nullptr;
    s.overhead = 0;
    s.checkpoints = 0;
    ++s.generation;

    // The cost of one timed call (two clock reads and the counter update), measured here so the
    // JSON can say how much of the counters' time is the counting.
    g_enabled.store(true, std::memory_order_relaxed);
    {
        const int cal = counter_id("profiler/calibration");
        constexpr int n = 200000;
        const uint64_t a = now_ns();
        for (int i = 0; i < n; ++i) {
            const CallTimer t(cal);
        }
        s.timer_pair_ns = double(now_ns() - a) / double(n);
    }
    zero_all_slots();

    // Where and when: the machine, the local start time, the process, the hardware threads.
    {
        std::string host = "unknown";
#if !defined(_WIN32)
        char buf[256] = {};
        if (gethostname(buf, sizeof(buf) - 1) == 0) host = buf;
        s.meta_json.emplace_back("pid", std::to_string(long(getpid())));
#endif
        s.meta_json.emplace_back("host", nlohmann::json(host).dump());
        const std::time_t now = std::time(nullptr);
        char tbuf[64] = {};
        std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%S", std::localtime(&now));
        s.meta_json.emplace_back("start_time", nlohmann::json(std::string(tbuf)).dump());
        s.meta_json.emplace_back(
            "hardware_concurrency",
            std::to_string(std::thread::hardware_concurrency()));
    }

    s.active = true;
    s.t0 = now_ns();
    uint64_t gen = 0;
    open_event(root_name, false, s.root, gen);
    m_uncaught = std::uncaught_exceptions();
    m_active = true;
    logger().info(
        "[profile] profiling to {} (one timed call costs {:.1f} ns)",
        path,
        s.timer_pair_ns);
}

Session::~Session()
{
    if (!m_active) return;
    State& s = st();
    const bool exception = std::uncaught_exceptions() > m_uncaught;
    const uint64_t e0 = now_ns();
    const uint64_t ovh0 = s.overhead;
    while (!s.stack.empty()) close_top(e0, ovh0);
    try {
        log_summary();
        write_json(exception ? "exception" : "ok");
        logger().info(
            "[profile] wrote {} ({} steps{})",
            s.path,
            s.events.size(),
            exception ? ", the run ended by an exception" : "");
    } catch (const std::exception& err) {
        logger().error("[profile] writing {} failed: {}", s.path, err.what());
    }
    g_enabled.store(false, std::memory_order_relaxed);
    s.active = false;
    s.probe = nullptr;
    s.owner.store(std::thread::id(), std::memory_order_relaxed);
}

void Session::meta_json(const std::string& key, const std::string& json_text)
{
    if (m_active) st().meta_json.emplace_back(key, json_text);
}

void Session::meta(const std::string& key, const std::string& value)
{
    if (m_active) st().meta_json.emplace_back(key, nlohmann::json(value).dump());
}

void Session::meta(const std::string& key, const double value)
{
    if (m_active) st().meta_json.emplace_back(key, nlohmann::json(value).dump());
}

void checkpoint()
{
    if (!enabled() || !on_owner()) return;
    State& s = st();
    if (!s.active) return;
    const uint64_t b0 = now_ns();
    ++s.checkpoints;
    try {
        write_json("running");
    } catch (const std::exception& err) {
        logger().warn("[profile] checkpoint of {} failed: {}", s.path, err.what());
    }
    s.overhead += now_ns() - b0;
}

} // namespace wmtk::prof
