// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The H.265 syntax a decoder that hands slices to the hardware reads itself:
// an Annex B access unit's NAL units, the parameter sets' ids and what they
// say about the pictures (size, crop, format, the VUI's colour, the reorder
// depth), and the start of a slice header, up to its picture order count.
// Pure host code, with no codec. Internal.

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "frame_color.hpp"

namespace volumetric_kit::recon::sensor::video::hevc {

// NAL unit types (H.265 Table 7-1) named here.
constexpr int kRaslN = 8;
constexpr int kRaslR = 9;
constexpr int kBlaWLp = 16;
constexpr int kIdrWRadl = 19;
constexpr int kIdrNLp = 20;
constexpr int kCraNut = 21;
constexpr int kVps = 32;
constexpr int kSps = 33;
constexpr int kPps = 34;
constexpr int kEos = 36;

// One NAL unit of an Annex B stream: its bytes from the header on, start
// code and trailing zeros off, emulation prevention left in.
struct Nal {
  const std::uint8_t* data = nullptr;
  std::size_t size = 0;
  int type = 0;
  int layer_id = 0;
  int temporal_id = 0;
};

// @p out given the NAL units of @p data, in order: false if a start code is
// not followed by a valid two-byte header, or there is no start code.
bool split_annex_b(const std::uint8_t* data, std::size_t size,
                   std::vector<Nal>& out);

inline bool is_irap(int type) noexcept { return type >= 16 && type <= 23; }
inline bool is_idr(int type) noexcept {
  return type == kIdrWRadl || type == kIdrNLp;
}
inline bool is_rasl(int type) noexcept {
  return type == kRaslN || type == kRaslR;
}

// A VPS's id (vps_video_parameter_set_id).
std::optional<std::uint32_t> vps_id(const Nal& vps);

// What a sequence parameter set says, read through its rbsp_trailing_bits.
struct Sps {
  std::uint32_t id = 0;
  std::uint32_t vps_id = 0;
  std::uint32_t chroma_format_idc = 1;
  bool separate_colour_plane = false;
  std::uint32_t coded_width = 0;  // luma samples
  std::uint32_t coded_height = 0;
  // The conformance window, in luma samples.
  std::uint32_t crop_left = 0;
  std::uint32_t crop_right = 0;
  std::uint32_t crop_top = 0;
  std::uint32_t crop_bottom = 0;
  std::uint32_t bit_depth_luma = 8;
  std::uint32_t bit_depth_chroma = 8;
  std::uint32_t log2_max_poc_lsb = 4;
  // sps_max_num_reorder_pics of the highest sub-layer.
  std::uint32_t max_num_reorder = 0;
  ColorCodes color;  // from the VUI; unspecified and limited without one

  std::uint32_t width() const noexcept {
    return coded_width - crop_left - crop_right;
  }
  std::uint32_t height() const noexcept {
    return coded_height - crop_top - crop_bottom;
  }
};

// The SPS @p sps holds; empty if it does not parse, its values are out of
// range, or anything follows it but its trailing bits.
std::optional<Sps> parse_sps(const Nal& sps);

// The leading fields of a picture parameter set.
struct Pps {
  std::uint32_t id = 0;
  std::uint32_t sps_id = 0;
  bool dependent_slice_segments_enabled = false;
  bool output_flag_present = false;
  std::uint32_t num_extra_slice_header_bits = 0;
};
std::optional<Pps> parse_pps(const Nal& pps);

// The start of a slice segment header.
struct SliceStart {
  bool first_slice_segment = false;
  bool no_output_of_prior_pics = false;
  std::uint32_t pps_id = 0;
  bool pic_output = true;
  std::uint32_t poc_lsb = 0;  // 0 for an IDR picture
};

// The start of @p slice's header, read with the parameter sets it names from
// @p pps and @p sps; empty if it does not parse or names one not there. Only
// the first segment of a picture is read past its PPS id.
std::optional<SliceStart> parse_slice_start(
    const Nal& slice, const std::map<std::uint32_t, Pps>& pps,
    const std::map<std::uint32_t, Sps>& sps);

}  // namespace volumetric_kit::recon::sensor::video::hevc
