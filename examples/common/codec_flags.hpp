// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/codec_flags.hpp
/// @brief The flags both codec examples take -- `--k`, `--step`,
///        `--quant-table`, `--entropy` and `--segment-size` -- into one
///        encoder configuration, validated once.

#include <string>

#include "cli.hpp"
#include "codec_quantization.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/codec/encoder.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// Parse @p text, the value of @p flag, as `auto`, `host` or `device`.
inline vkc::Status parse_entropy(const std::string& flag,
                                 const std::string& text,
                                 vr::codec::EntropyCoding& out) {
  if (text == "auto") {
    out = vr::codec::EntropyCoding::kAuto;
  } else if (text == "host") {
    out = vr::codec::EntropyCoding::kHost;
  } else if (text == "device") {
    out = vr::codec::EntropyCoding::kDevice;
  } else {
    return vkc::Status::invalid_argument(flag + " needs auto, host or device");
  }
  return {};
}

/// @brief A codec example's encoder, as its command line configured it.
struct CodecFlags {
  vr::codec::EncoderConfig config;      ///< `--k`, `--step`, ...
  std::string quant_table = "uniform";  ///< `apply_quantization_table`'s.

  /// @brief Declare the five flags on @p cli, and the check that applies the
  ///        table and validates the parameters. This object must outlive the
  ///        parse.
  void add_to(Cli& cli) {
    cli.option("--k", "N", config.params.coefficient_count, 1)
        .option("--step", "f", config.params.quantization_scale)
        .option("--quant-table", "uniform|band|radial", quant_table)
        .on("--entropy", "auto|host|device",
            [this](const std::string& flag, const char* value) {
              return parse_entropy(flag, value, config.entropy);
            })
        .option("--segment-size", "N", config.segment_size, 1)
        .check([this]() -> vkc::Status {
          VKC_TRY(apply_quantization_table(config.params, quant_table));
          return config.params.validate();
        });
  }
};

}  // namespace vr_example
