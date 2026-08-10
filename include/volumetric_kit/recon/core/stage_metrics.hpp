// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file core/stage_metrics.hpp
/// @brief Named CPU/GPU spans: the vocabulary every tier reports timings in and
///        every consumer displays.
///
/// Plain data with no Vulkan and no allocator in sight, so a pure-host consumer
/// -- an exporter, a benchmark, a codec tier -- reads it without preprocessing
/// a driver header, and a renderer maps it into its own metrics shape with a
/// loop. That mapping, rather than a shared package, is deliberate: see the
/// class note.

#include <chrono>
#include <cstddef>
#include <cstring>
#include <vector>

namespace volumetric_kit::recon {

/// @brief One labelled stage: its host span, and -- where a device timer ran --
///        its GPU span.
///
/// @ref has_gpu is what a consumer branches on to *display* a device column: it
/// reads @ref gpu_ms only where this is set, with no separate "does this device
/// support timestamps" flag to consult. But read it as "a device span was
/// measured for this row", which is the only thing it can mean, and not as a
/// capability report -- it is false for all of:
///
/// - a stage that is genuinely host-only (a dataset read, an atlas repack);
/// - a call that returned before dispatching anything (an empty active set, a
///   refused argument) -- the host row is still charged, deliberately, since a
///   stage silent on failure reads as one that did not run;
/// - every stage on a queue family whose `timestampValidBits` is zero, or whose
///   query pool would not allocate.
///
/// Telling those apart is not this struct's job and cannot be done from one
/// bool. A consumer that must -- a test asserting the device half is really
/// there -- asks the device itself, by creating a @ref GpuTimer and reading
/// `available()`. A capability that can be established independently must never
/// be inferred from the thing under test.
struct StageRow {
  /// Stage label. Stored **by pointer**, not copied, which is what keeps this
  /// trivially copyable -- so it must have string-literal lifetime (or point at
  /// storage outliving every read of the metrics). A pointer into a temporary
  /// dangles.
  const char* name = nullptr;
  /// Wall-clock host time, in milliseconds. Always populated.
  ///
  /// For a recon compute stage this is genuinely *wall clock around a blocking
  /// submit*: `Device::submit_single_time` waits its fence, so the span covers
  /// host record, submit, the fence stall, and device execution together. It is
  /// therefore an honest end-to-end cost and **not** a host-only measurement --
  /// which is exactly why @ref gpu_ms exists to separate them.
  double cpu_ms = 0.0;
  /// Device time, in milliseconds. Meaningful only when @ref has_gpu is set;
  /// otherwise it is `0.0` and carries no information.
  double gpu_ms = 0.0;
  /// Whether @ref gpu_ms holds a real timestamp measurement.
  bool has_gpu = false;
};

/// @brief An accumulating set of named spans, in first-seen order.
///
/// The opt-in reporting surface the tiers share. A tier takes
/// `StageMetrics* = nullptr` on its entry point and measures nothing when the
/// caller passes null -- the shape `mesh::ExtractTimings` established and the
/// 2026-08-01 decision blessed: no global sink, no state retained between
/// calls, no dependency added, nothing measured when unasked.
///
/// **Why this is recon's own type rather than the renderer's.** The renderer
/// already publishes a structurally identical `gfx::FrameMetrics::Section`, and
/// nothing about that struct is renderer-specific -- but recon cannot include
/// from a sibling it does not depend on, and extracting a shared package for
/// four fields was weighed and judged too thin. So each side declares in its
/// own vocabulary and the neutral app that already knows both maps one to the
/// other in a loop. That is the same create/adopt shape as `DeviceRequirements`
/// and `MarchingCubesConfig::extra_vertex_usage`, and it keeps either release
/// from coupling to the other.
///
/// Repeat calls under one name **accumulate**, so a stage that runs more than
/// once for a frame (an allocate retried across a map resize, a meshing
/// dispatch re-run after a refit) reports its total rather than several rows or
/// only the last.
///
/// @code
/// StageMetrics m;
/// m.seed("texture");                 // a row that may not run this frame
/// {
///   StageScope s(m, "integrate");    // times until the scope closes
///   integrator.integrate(frame);
/// }
/// timer.report_into(m);              // GPU halves, where a GpuTimer ran
/// for (const StageRow& row : m.rows()) { ... }
/// @endcode
class StageMetrics {
 public:
  /// @brief Prefix marking a row as a *breakdown* of the row above it rather
  ///        than a stage of its own -- the phases inside one extract, say.
  ///
  /// Indented so a table reads as a hierarchy, and skipped by @ref
  /// total_cpu_ms so the sum reads as one too. Without that, a flat list
  /// carrying a two-level structure double-counts every decomposed stage --
  /// which is not hypothetical: it reported every extract twice in the viewer
  /// before the skip existed.
  ///
  /// @ref total_gpu_ms deliberately does **not** skip them, because the two
  /// halves nest differently: a host scope spans a whole call including
  /// everything it invokes, while a device span covers one dispatch. A
  /// breakdown row carrying a GPU measurement is therefore always a *separate*
  /// dispatch that no row above it contains, and skipping it drops that time
  /// from every total rather than deduplicating it.
  static constexpr const char* kBreakdownPrefix = "  ..";

  /// @brief Drop every row, keeping the storage for reuse.
  void clear() noexcept { rows_.clear(); }

  /// @brief Add @p milliseconds of host time to @p name's row, creating it if
  ///        new.
  /// @param name          Stage label; string-literal lifetime (see @ref
  ///                      StageRow::name).
  /// @param milliseconds  The span to add.
  void add_cpu(const char* name, double milliseconds) {
    find_or_add(name).cpu_ms += milliseconds;
  }

  /// @brief Add @p milliseconds of device time to @p name's row, creating it if
  ///        new, and mark the row as carrying a GPU measurement.
  ///
  /// Separate from @ref add_cpu rather than a second argument to it, because
  /// the two halves are measured by different mechanisms at different times: a
  /// host scope closes when the call returns, while a GPU span is only readable
  /// after the submit's fence. A row may therefore carry a host span and gain
  /// its device span later, in either order.
  ///
  /// @param name          Stage label; string-literal lifetime.
  /// @param milliseconds  The span to add.
  void add_gpu(const char* name, double milliseconds) {
    StageRow& row = find_or_add(name);
    row.gpu_ms += milliseconds;
    row.has_gpu = true;
  }

  /// @brief Create @p name's row at zero if it does not exist yet.
  ///
  /// Seeding every stage a frame *might* run keeps a table's shape stable: a
  /// stage that did not run this time (the extract on a frame between remeshes)
  /// reports 0.00 rather than dropping its row and making every row below it
  /// jump, which is what makes a live read-out unreadable.
  void seed(const char* name) { find_or_add(name); }

  /// @brief Fold @p other's rows into this one, matched by name.
  ///
  /// Both halves of a row carry over, which is the whole reason this lives on
  /// the vocabulary type rather than in each consumer: a hand-written merge
  /// loop reads naturally as `add_cpu(row.name, row.cpu_ms)` and silently
  /// drops @ref StageRow::gpu_ms and @ref StageRow::has_gpu, and the loss is
  /// invisible downstream because an absent `has_gpu` is indistinguishable
  /// from a genuinely host-only stage.
  ///
  /// Rows arriving from @p other keep pointing at *its* labels, so those must
  /// outlive every read of this set (see @ref StageRow::name).
  ///
  /// @param other  The set to fold in; merging a set into itself does nothing.
  void merge(const StageMetrics& other) {
    if (this == &other) return;
    for (const StageRow& row : other.rows_) {
      StageRow& dst = find_or_add(row.name);
      dst.cpu_ms += row.cpu_ms;
      if (row.has_gpu) {
        dst.gpu_ms += row.gpu_ms;
        dst.has_gpu = true;
      }
    }
  }

  /// @return The recorded rows, in first-seen order.
  const std::vector<StageRow>& rows() const noexcept { return rows_; }

  /// @return `true` when nothing has been recorded.
  bool empty() const noexcept { return rows_.empty(); }

  /// @brief Sum the rows' host milliseconds, skipping breakdowns.
  ///
  /// @param exclude  A further label to leave out, matched by content like @ref
  ///                 add_cpu; `nullptr` excludes only the breakdown rows. A
  ///                 caller uses it for a row that is timed for visibility but
  ///                 is not part of the figure it reports -- a dataset read
  ///                 shown beside fusion stages without inflating "fuse
  ///                 ms/frame".
  /// @return The summed milliseconds.
  double total_cpu_ms(const char* exclude = nullptr) const noexcept {
    double total = 0.0;
    for (const StageRow& row : rows_) {
      if (counts_toward_total(row, exclude)) total += row.cpu_ms;
    }
    return total;
  }

  /// @brief Sum the rows' device milliseconds, breakdown rows **included**.
  ///
  /// Unlike @ref total_cpu_ms this counts breakdowns, because a device span
  /// measures one dispatch rather than one call and so is never contained in
  /// the span of the row above it -- see @ref kBreakdownPrefix. Skipping them
  /// here lost the whole of any sub-row whose GPU half is a second kernel,
  /// while the host half of that same work stayed counted through its parent.
  ///
  /// Rows with no GPU measurement contribute nothing, so this is the device
  /// total of the stages that *could* be measured -- not comparable with
  /// @ref total_cpu_ms unless every row carries a GPU span.
  ///
  /// @param exclude  As @ref total_cpu_ms.
  /// @return The summed milliseconds.
  double total_gpu_ms(const char* exclude = nullptr) const noexcept {
    double total = 0.0;
    for (const StageRow& row : rows_) {
      if (!row.has_gpu) continue;
      if (exclude != nullptr && std::strcmp(row.name, exclude) == 0) continue;
      total += row.gpu_ms;
    }
    return total;
  }

  /// @return `true` while a @ref StageScope is open on this set -- so a row
  ///         added now is nested inside a stage whose host span contains it.
  ///
  /// What an operation called from both positions needs in order to name its
  /// row: whether that row is a breakdown (@ref kBreakdownPrefix) or a stage of
  /// its own is a property of *where it was called*, not of what it does. A
  /// callee that hard-codes the prefix hands a top-level caller a row that
  /// @ref total_cpu_ms skips -- their only stage, missing from their own total,
  /// while still sitting in @ref rows() looking accounted for.
  bool in_stage() const noexcept { return open_scopes_ > 0; }

 private:
  // Rows are matched by string CONTENT, not pointer. Seeding a row and timing
  // it name the same stage from two places in the source, and identical string
  // literals are not guaranteed to be pooled to one address -- across
  // translation units they routinely are not, which would silently split one
  // stage into two rows that each look plausible.
  StageRow& find_or_add(const char* name) {
    for (StageRow& row : rows_) {
      if (std::strcmp(row.name, name) == 0) return row;
    }
    rows_.push_back(StageRow{name, 0.0, 0.0, false});
    return rows_.back();
  }

  static bool counts_toward_total(const StageRow& row,
                                  const char* exclude) noexcept {
    if (exclude != nullptr && std::strcmp(row.name, exclude) == 0) return false;
    return std::strncmp(row.name, kBreakdownPrefix,
                        std::strlen(kBreakdownPrefix)) != 0;
  }

  // Maintained by StageScope alone, so @ref in_stage answers from the scopes
  // actually open rather than from each callee being told where it stands.
  void push_scope() noexcept { ++open_scopes_; }
  void pop_scope() noexcept { --open_scopes_; }

  friend class StageScope;

  std::vector<StageRow> rows_;
  int open_scopes_ = 0;
};

/// @brief Times its own scope into a @ref StageMetrics row.
///
/// The pointer constructor is what makes "nothing is measured when the caller
/// passes null" true at the call site rather than by convention: a tier's
/// reporting out-param is `StageMetrics* = nullptr`, and a scope that cannot be
/// conditionally constructed would otherwise need an `if (metrics)` around
/// every timed site. Null is inert, so the null check in the destructor is the
/// same one both constructors rely on rather than dead code beside a second
/// copy of this class.
///
/// @warning The @ref StageMetrics and the @p name literal must both outlive the
///          scope.
///
/// @code
/// {
///   StageScope s(metrics, "allocate");            // a metrics you hold
///   VR_TRY(grid.allocate_from_depth(depth, camera));
/// }  // the span lands in metrics here
///
/// Status TsdfIntegrator::integrate(..., StageMetrics* metrics) {
///   StageScope s(metrics, "integrate");           // inert when null
///   ...
/// }
/// @endcode
class StageScope {
 public:
  /// @brief Start timing @p name into @p metrics.
  StageScope(StageMetrics& metrics, const char* name)
      : StageScope(&metrics, name) {}

  /// @brief Start timing @p name into @p metrics, or do nothing if it is null.
  StageScope(StageMetrics* metrics, const char* name)
      : metrics_(metrics), name_(name), start_(Clock::now()) {
    if (metrics_ != nullptr) metrics_->push_scope();
  }

  /// @brief Stop timing and add the elapsed span to the row, unless inert.
  ~StageScope() {
    if (metrics_ == nullptr) return;
    // Closed before the row is added, so a scope opened by whatever runs next
    // sees the nesting it is actually in (@ref StageMetrics::in_stage).
    metrics_->pop_scope();
    metrics_->add_cpu(
        name_, std::chrono::duration<double, std::milli>(Clock::now() - start_)
                   .count());
  }

  StageScope(const StageScope&) = delete;
  StageScope& operator=(const StageScope&) = delete;
  StageScope(StageScope&&) = delete;
  StageScope& operator=(StageScope&&) = delete;

  /// @return The row this times into; see @ref StageRow::name for its lifetime.
  const char* name() const noexcept { return name_; }

 private:
  using Clock = std::chrono::steady_clock;

  StageMetrics* metrics_;
  const char* name_;
  Clock::time_point start_;
};

}  // namespace volumetric_kit::recon
