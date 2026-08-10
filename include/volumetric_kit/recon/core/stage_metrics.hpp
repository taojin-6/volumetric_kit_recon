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
/// @ref has_gpu **is** the capability report. A consumer never branches on a
/// separate "does this device support timestamps" flag; it reads @ref gpu_ms
/// only where this is set, which is false both for a stage that is genuinely
/// host-only and for every stage on a queue family whose `timestampValidBits`
/// is zero.
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
  /// total_cpu_ms / @ref total_gpu_ms so the sum reads as one too. Without
  /// that, a flat list carrying a two-level structure double-counts every
  /// decomposed stage -- which is not hypothetical: it reported every extract
  /// twice in the viewer before the skip existed.
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

  /// @brief Sum the rows' device milliseconds, skipping breakdowns.
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
      if (row.has_gpu && counts_toward_total(row, exclude)) total += row.gpu_ms;
    }
    return total;
  }

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

  std::vector<StageRow> rows_;
};

/// @brief Times its own scope into a @ref StageMetrics row.
///
/// @warning The @ref StageMetrics and the @p name literal must both outlive the
///          scope.
///
/// @code
/// {
///   StageScope s(metrics, "allocate");
///   VR_TRY(grid.allocate_from_depth(depth, camera));
/// }  // the span lands in metrics here
/// @endcode
class StageScope {
 public:
  /// @brief Start timing @p name into @p metrics.
  StageScope(StageMetrics& metrics, const char* name)
      : metrics_(&metrics), name_(name), start_(Clock::now()) {}

  /// @brief Stop timing and add the elapsed span to the row.
  ~StageScope() {
    if (metrics_ == nullptr) return;
    metrics_->add_cpu(
        name_, std::chrono::duration<double, std::milli>(Clock::now() - start_)
                   .count());
  }

  StageScope(const StageScope&) = delete;
  StageScope& operator=(const StageScope&) = delete;
  StageScope(StageScope&&) = delete;
  StageScope& operator=(StageScope&&) = delete;

 private:
  using Clock = std::chrono::steady_clock;

  StageMetrics* metrics_;
  const char* name_;
  Clock::time_point start_;
};

/// @brief A @ref StageScope that does nothing when the caller opted out.
///
/// Every tier's reporting out-param is `StageMetrics* = nullptr`, so every
/// timed site would otherwise need `if (metrics) { ... }` around a scope that
/// cannot be conditionally constructed. This takes the pointer directly and is
/// inert when it is null -- which is what makes "nothing is measured when the
/// caller passes null" true at the call site rather than by convention.
///
/// @code
/// Status TsdfIntegrator::integrate(..., StageMetrics* metrics) {
///   OptionalStageScope s(metrics, "integrate");   // inert when null
///   ...
/// }
/// @endcode
class OptionalStageScope {
 public:
  /// @brief Start timing @p name into @p metrics, or do nothing if it is null.
  OptionalStageScope(StageMetrics* metrics, const char* name)
      : metrics_(metrics), name_(name), start_(Clock::now()) {}

  /// @brief Stop timing and add the span, unless inert.
  ~OptionalStageScope() {
    if (metrics_ == nullptr) return;
    metrics_->add_cpu(
        name_, std::chrono::duration<double, std::milli>(Clock::now() - start_)
                   .count());
  }

  OptionalStageScope(const OptionalStageScope&) = delete;
  OptionalStageScope& operator=(const OptionalStageScope&) = delete;
  OptionalStageScope(OptionalStageScope&&) = delete;
  OptionalStageScope& operator=(OptionalStageScope&&) = delete;

 private:
  using Clock = std::chrono::steady_clock;

  StageMetrics* metrics_;
  const char* name_;
  Clock::time_point start_;
};

}  // namespace volumetric_kit::recon
