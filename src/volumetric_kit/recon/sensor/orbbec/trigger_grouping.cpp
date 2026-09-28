// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "trigger_grouping.hpp"

#include <algorithm>
#include <limits>

namespace volumetric_kit::recon::sensor::orbbec {

Result<std::vector<std::size_t>> rig_start_order(
    const std::vector<OrbbecSyncMode>& modes,
    const std::vector<std::string>& serials) {
  std::vector<std::size_t> order;
  std::optional<std::size_t> primary;
  std::string roles;
  for (std::size_t i = 0; i < modes.size(); ++i) {
    roles += (i == 0 ? "" : ", ") + serials[i] + " " + to_string(modes[i]);
  }
  for (std::size_t i = 0; i < modes.size(); ++i) {
    if (modes[i] == OrbbecSyncMode::Primary) {
      if (primary) {
        return Status::unsupported("OrbbecRig: more than one sync primary (" +
                                   roles + ")");
      }
      primary = i;
    } else if (waits_for_primary(modes[i])) {
      order.push_back(i);
    } else {
      return Status::unsupported(
          "OrbbecRig: camera " + serials[i] + " is " + to_string(modes[i]) +
          ", so it streams on its own clock rather than on the primary's "
          "trigger (" +
          roles + ")");
    }
  }
  if (!primary) {
    return Status::unsupported(
        "OrbbecRig: no sync primary, so nothing triggers the others (" + roles +
        ")");
  }
  order.push_back(*primary);
  return order;
}

TriggerGrouper::TriggerGrouper(const Config& config)
    : config_(config), queues_(config.cameras) {}

void TriggerGrouper::add(std::size_t camera, std::uint64_t ts_us,
                         std::uint64_t id, std::uint64_t now_us,
                         std::vector<std::uint64_t>* released) {
  std::deque<Entry>& q = queues_[camera];
  // Kept in timestamp order: a clock re-sync can step a camera's clock back.
  auto at = q.end();
  while (at != q.begin() && std::prev(at)->ts_us > ts_us) --at;
  q.insert(at, Entry{ts_us, id, now_us});
  while (q.size() > config_.queue_depth) {
    released->push_back(q.front().id);
    q.pop_front();
  }
}

std::optional<TriggerGrouper::Group> TriggerGrouper::take(
    std::uint64_t now_us, std::vector<std::uint64_t>* released) {
  std::vector<Group> ready;
  for (;;) {
    // The earliest trigger held: the earliest front, and every front within
    // the tolerance of it.
    std::uint64_t t0 = std::numeric_limits<std::uint64_t>::max();
    for (const auto& q : queues_) {
      if (!q.empty()) t0 = std::min(t0, q.front().ts_us);
    }
    if (t0 == std::numeric_limits<std::uint64_t>::max()) break;
    Group group;
    group.ids.resize(queues_.size());
    group.timestamp_us = t0;
    std::uint64_t first_arrival = std::numeric_limits<std::uint64_t>::max();
    bool waiting = false;
    for (std::size_t c = 0; c < queues_.size(); ++c) {
      const auto& q = queues_[c];
      if (!q.empty() && q.front().ts_us - t0 <= config_.tolerance_us) {
        group.ids[c] = q.front().id;
        first_arrival = std::min(first_arrival, q.front().arrived_us);
        if (c == config_.anchor) group.timestamp_us = q.front().ts_us;
      } else if (q.empty()) {
        // Silent: its frame for this trigger may still be on the way. (A
        // non-empty queue here holds a later frame, so this camera is past
        // the trigger and will never send one for it.)
        waiting = true;
      }
    }
    const std::uint64_t waited =
        now_us > first_arrival ? now_us - first_arrival : 0;
    if (waiting && waited < config_.max_wait_us) break;
    for (std::size_t c = 0; c < queues_.size(); ++c) {
      if (group.ids[c]) queues_[c].pop_front();
    }
    ready.push_back(std::move(group));
  }
  if (ready.empty()) return std::nullopt;

  // The newest ready trigger holding the anchor, else the newest.
  std::size_t pick = ready.size() - 1;
  for (std::size_t i = ready.size(); i-- > 0;) {
    if (ready[i].ids[config_.anchor]) {
      pick = i;
      break;
    }
  }
  for (std::size_t i = 0; i < ready.size(); ++i) {
    if (i == pick) continue;
    for (const auto& id : ready[i].ids) {
      if (id) released->push_back(*id);
    }
  }
  return std::move(ready[pick]);
}

void TriggerGrouper::clear(std::vector<std::uint64_t>* released) {
  for (auto& q : queues_) {
    for (const Entry& e : q) released->push_back(e.id);
    q.clear();
  }
}

}  // namespace volumetric_kit::recon::sensor::orbbec
