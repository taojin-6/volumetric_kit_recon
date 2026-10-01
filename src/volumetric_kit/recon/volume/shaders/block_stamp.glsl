// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// One block slot's stamps, ticks of the map's clock (mirrors
// volume::BlockStamp in hash_types.hpp). Its own header so a kernel in any
// tier can stamp a block without taking hash_common.glsl's other structs.

#ifndef VR_BLOCK_STAMP_GLSL
#define VR_BLOCK_STAMP_GLSL

struct BlockStamp {
  uint requested;
  uint weighted;
  uint changed;
};

// Whether tick a is later than tick b, modulo 2^32 (volume::tick_after).
bool tick_after(uint a, uint b) { return int(a - b) > 0; }

#endif
