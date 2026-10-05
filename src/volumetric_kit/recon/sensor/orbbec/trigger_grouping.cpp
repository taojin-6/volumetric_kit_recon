// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "trigger_grouping.hpp"

#include <algorithm>

namespace volumetric_kit::recon::sensor::orbbec {

core::Result<std::vector<std::size_t>> rig_start_order(
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
        return core::Status::unsupported(
            "OrbbecRig: more than one sync primary (" + roles + ")");
      }
      primary = i;
    } else if (waits_for_primary(modes[i])) {
      order.push_back(i);
    } else {
      return core::Status::unsupported(
          "OrbbecRig: camera " + serials[i] + " is " + to_string(modes[i]) +
          ", so it streams on its own clock rather than on the primary's "
          "trigger (" +
          roles + ")");
    }
  }
  if (!primary) {
    return core::Status::unsupported(
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
  std::deque<Entry>& anchor = queues_[config_.anchor];
  std::optional<Group> newest;
  while (!anchor.empty()) {
    // The earliest anchor frame held names a trigger; each other camera's
    // front frame within the tolerance of it is that camera's frame for it.
    const Entry& trigger = anchor.front();
    Group group;
    group.timestamp_us = trigger.ts_us;
    group.ids.resize(queues_.size());
    group.ids[config_.anchor] = trigger.id;
    std::uint64_t first_arrival = trigger.arrived_us;
    bool waiting = false;
    for (std::size_t c = 0; c < queues_.size(); ++c) {
      if (c == config_.anchor) continue;
      std::deque<Entry>& q = queues_[c];
      // Earlier than the tolerance allows: near no anchor frame still to come
      // -- its set already went out, the anchor missed that trigger, or this
      // camera's clock is off.
      while (!q.empty() &&
             q.front().ts_us + config_.tolerance_us < trigger.ts_us) {
        released->push_back(q.front().id);
        q.pop_front();
      }
      if (q.empty()) {
        // Silent: its frame for this trigger may still be on the way.
        waiting = true;
      } else if (q.front().ts_us <= trigger.ts_us + config_.tolerance_us) {
        group.ids[c] = q.front().id;
        first_arrival = std::min(first_arrival, q.front().arrived_us);
      }
      // Otherwise it holds a later frame: past the trigger, it will never
      // send one for it.
    }
    const std::uint64_t waited =
        now_us > first_arrival ? now_us - first_arrival : 0;
    if (waiting && waited < config_.max_wait_us) break;
    for (std::size_t c = 0; c < queues_.size(); ++c) {
      if (group.ids[c]) queues_[c].pop_front();
    }
    if (newest) {
      for (const auto& id : newest->ids) {
        if (id) released->push_back(*id);
      }
    }
    newest = std::move(group);
  }
  return newest;
}

void TriggerGrouper::clear(std::vector<std::uint64_t>* released) {
  for (auto& q : queues_) {
    for (const Entry& e : q) released->push_back(e.id);
    q.clear();
  }
}

}  // namespace volumetric_kit::recon::sensor::orbbec
