// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec's set cut (examples/common/orbbec_rig.hpp): a run of N frames
// keeps a set's first frames in the array's order, past its missing ones, and
// lets the rest go, their pictures with them. Host only.

#include <cstddef>
#include <cstdio>
#include <memory>
#include <utility>
#include <vector>

#include "orbbec_rig.hpp"
#include "test_check.hpp"

namespace sensor = volumetric_kit::recon::sensor;

namespace {

// A set with a frame from each camera `present` names, numbered by camera,
// each holding a picture `pictures` watches.
sensor::FrameSet set_of(const std::vector<bool>& present,
                        std::vector<std::weak_ptr<const void>>* pictures) {
  sensor::FrameSet set;
  set.frames.resize(present.size());
  for (std::size_t c = 0; c < present.size(); ++c) {
    if (!present[c]) continue;
    sensor::RgbdFrame frame;
    frame.sequence = c;
    auto picture = std::make_shared<int>(0);
    pictures->push_back(picture);
    frame.pixels = std::move(picture);
    set.frames[c] = std::move(frame);
  }
  return set;
}

int cut() {
  std::vector<std::weak_ptr<const void>> pictures;
  // Room for the whole set: every frame stays.
  sensor::FrameSet set = set_of({true, false, true, true}, &pictures);
  CHECK(vr_example::cut_set(set, 5) == 3 && set.count() == 3);
  CHECK(vr_example::cut_set(set, 3) == 3 && set.count() == 3);

  // The first two, in the array's order, past the missing one; the third
  // goes, and its picture with it.
  pictures.clear();
  set = set_of({true, false, true, true}, &pictures);
  CHECK(vr_example::cut_set(set, 2) == 2 && set.frames.size() == 4);
  CHECK(set.frames[0] && set.frames[0]->sequence == 0 && !set.frames[1]);
  CHECK(set.frames[2] && set.frames[2]->sequence == 2 && !set.frames[3]);
  CHECK(!pictures[0].expired() && !pictures[1].expired());
  CHECK(pictures[2].expired());

  // None left: the set keeps a slot per camera, with no frame.
  pictures.clear();
  set = set_of({true, true}, &pictures);
  CHECK(vr_example::cut_set(set, 0) == 0 && set.frames.size() == 2);
  CHECK(set.count() == 0 && pictures[0].expired() && pictures[1].expired());
  return 0;
}

}  // namespace

int main() {
  CHECK(cut() == 0);
  std::puts("example_orbbec_rig: OK");
  return 0;
}
