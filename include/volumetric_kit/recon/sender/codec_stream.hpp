// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sender/codec_stream.hpp
/// @brief A sender's codec stream: each frame encoded, written to a `.vrst`
///        file, served to live viewers, and decoded back as a viewer would.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/codec/codec_params.hpp"
#include "volumetric_kit/recon/codec/encoder.hpp"
#include "volumetric_kit/recon/core/fwd.hpp"
#include "volumetric_kit/recon/networking/stream_server.hpp"
#include "volumetric_kit/recon/sender/export.hpp"
#include "volumetric_kit/recon/stream/session.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace volumetric_kit::recon::sender {

namespace detail {
class StreamCoder;
}  // namespace detail

/// @brief Where a @ref CodecStream's frames go, each as it is coded.
struct CodecOutputs {
  /// A new `.vrst` file (stream/session.hpp) of every frame the encoder
  /// returns, a record each, an I-frame's a key; empty writes none. Nothing
  /// may be at the path, and its directory must exist.
  std::string stream;
  /// Where to serve the same records live, as they are recorded
  /// (@ref networking::StreamServer, on the stream's session); none serves
  /// nothing. A viewer's request for a key is met at the next frame, so it
  /// may serve without `replay_gop`. Defaulted, so one-field braces stay
  /// quiet under -Wmissing-field-initializers.
  std::optional<networking::ServerOptions> serve = std::nullopt;
  /// Decode each frame back into @ref CodecStream::player, as a viewer
  /// does, for @ref SentFrame::decode. Off, no decoder or player grid is
  /// made.
  bool decode = true;
};

/// @brief A frame as it went out, from @ref CodecStream::code or
///        @ref SetSender::send.
struct SentFrame {
  /// An I-frame, or a P-frame on the frame coded before.
  codec::FrameType type = codec::FrameType::kIntra;
  std::size_t bytes = 0;     ///< Its record's payload, the compressed frame.
  std::uint32_t blocks = 0;  ///< The blocks it carries.
  /// Its payload outgrew what its batch copied and took a tail
  /// (`"  ..tail"`, @ref codec::Encoder::finish).
  bool tail = false;
  /// The encode's stage rows; a sender's frame's, both halves'.
  core::StageMetrics encode;
  /// The decode's stage rows; empty without @ref CodecOutputs::decode.
  core::StageMetrics decode;
};

/// @brief Codes grids as frames (@ref codec::Encoder): each written to a
///        `.vrst` file and served live as @ref CodecOutputs says, then
///        decoded back into a player grid, as a viewer decodes it.
///
/// The file and the viewers get one session, the frames records of its
/// track 0. The player grid is built from the first frame's header and grows
/// as later frames need (codec/decode_grid.hpp). The encoder codes its next
/// P-frame against every frame it returns, so any failure after an encode
/// makes the next frame an I-frame (@ref codec::Encoder::force_intra): a
/// frame the file or the player lacks is never a reference. A viewer joining
/// or falling back makes the next frame an I-frame too
/// (@ref networking::StreamServer::key_wanted), as it waits for a key: the
/// request stands until a frame is returned, and joins between two frames
/// make one key.
///
/// @code
/// VKC_ASSIGN(sender::CodecStream stream,
///            sender::CodecStream::create(device, allocator, config, session,
///                                        outputs));
/// for (std::uint64_t set = 0; set < sets; ++set) {
///   fuse(grid, set);
///   VKC_ASSIGN(const sender::SentFrame sent,
///              stream.code(grid, set, pts_ns(set)));
/// }
/// VKC_TRY(stream.finish());
/// @endcode
class VR_SENDER_API CodecStream {
 public:
  /// @brief Make the outputs, then the encoder and, with
  ///        @ref CodecOutputs::decode, the decoder.
  ///
  /// The server first, so a port in use touches no file, then the file,
  /// whose rerun onto an existing one is the common refusal; the codec's
  /// kernels last, so a refused output costs none.
  /// @param device     The compute device (must outlive the stream).
  /// @param allocator  The allocator its buffers come from (must outlive the
  ///                   stream).
  /// @param config     The encoder's settings.
  /// @param session    The file's and the viewers' session, written as given:
  ///                   the frames are records of its track 0, a Geometry
  ///                   track. With no id or start time, a rerun writes the
  ///                   same bytes.
  /// @param outputs    Where the frames go.
  /// @return The stream; `Status::Code::InvalidArgument` for a session whose
  ///         track 0 is not a Geometry track; what
  ///         @ref networking::StreamServer::start returns
  ///         (`Status::Code::IoError` for a port in use); what
  ///         @ref stream::StreamWriter::create returns
  ///         (`Status::Code::InvalidArgument` when something is at the path,
  ///         `Status::Code::NotFound` when its directory is missing); or the
  ///         encoder's or the decoder's `create` error, the new file removed
  ///         again.
  static core::Result<CodecStream> create(core::Device& device,
                                          core::Allocator& allocator,
                                          const codec::EncoderConfig& config,
                                          const stream::Session& session,
                                          CodecOutputs outputs = {});

  CodecStream(CodecStream&& other) noexcept;
  CodecStream& operator=(CodecStream&&) = delete;
  /// Closes the viewers and the file. A file never finished that holds no
  /// record, as after a run that failed before its first frame, is removed,
  /// so a rerun onto it is not refused; one that holds records stays, read as
  /// a killed writer's.
  ~CodecStream();

  /// @brief Encode @p grid as one frame, write it, serve it, then decode it
  ///        back.
  ///
  /// Written as soon as it is encoded, so a frame the decoder refuses is
  /// kept; served once written, so no viewer gets a frame the file lacks. A
  /// frame the server refuses is logged at Warning and fails nothing: the
  /// viewers lose the rest of its GOP, the file nothing.
  /// @param grid    The grid to code.
  /// @param set     Its record's set: above every frame's coded before.
  /// @param pts_ns  Its record's pts: no earlier than the frames' coded
  ///                before.
  /// @return The frame; `Status::Code::InvalidArgument`, before anything is
  ///         encoded, for a @p set or @p pts_ns out of order or a finished or
  ///         moved-from stream; the encode's error; what
  ///         @ref stream::StreamWriter::append returns; or the decode's
  ///         error, the frame written and counted by @ref coded.
  core::Result<SentFrame> code(volume::VoxelBlockGrid& grid, std::uint64_t set,
                               std::uint64_t pts_ns);

  /// @brief Close the viewers, which see the stream end, and finish the file
  ///        with its footer index (@ref stream::StreamWriter::finish);
  ///        without either, only mark the stream finished. One attempt:
  ///        however it ends, every later call, this one included, is refused,
  ///        so a file whose footer failed stays unfinished, read as a killed
  ///        writer's.
  /// @return OK; `Status::Code::InvalidArgument` once finished or on a
  ///         moved-from stream; or the writer's error.
  core::Status finish();

  /// @return The grid the frames decode into, holding the last decoded; null
  ///         before the first frame, without @ref CodecOutputs::decode, and
  ///         on a moved-from stream.
  volume::VoxelBlockGrid* player() noexcept;

  /// @return The server the frames are served on; null when not serving,
  ///         once finished, and on a moved-from stream.
  const networking::StreamServer* server() const noexcept;

  /// @return The pts of the last frame written, below which @ref code
  ///         refuses one; 0 before the first.
  std::uint64_t last_pts_ns() const noexcept;

  /// @return The frames encoded and written so far, one the decoder refused
  ///         among them; the destructor removes an unfinished stream's file
  ///         only while this is 0.
  std::uint64_t coded() const noexcept;

  /// @return What codes the frames, for @ref SetSender, which records a
  ///         frame in halves around its set's fusion. A type the library
  ///         completes only inside itself; null on a moved-from stream.
  detail::StreamCoder* coder() noexcept;

 private:
  explicit CodecStream(std::unique_ptr<detail::StreamCoder> coder) noexcept;

  std::unique_ptr<detail::StreamCoder> coder_;
};

}  // namespace volumetric_kit::recon::sender
